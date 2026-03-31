import base64
import logging
import struct
from typing import Dict, Optional

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError

from . import config
from .common import mark_rx, utc_now

try:
    from meshtastic import mesh_pb2
except ImportError:
    mesh_pb2 = None


class FrameReader:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= 4:
            start = self._find_start()
            if start < 0:
                self.buffer.clear()
                break
            if start > 0:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            length = struct.unpack('>H', self.buffer[2:4])[0]
            frame_len = 4 + length
            if len(self.buffer) < frame_len:
                break
            payload = bytes(self.buffer[4:frame_len])
            del self.buffer[:frame_len]
            frames.append(payload)
        return frames

    def _find_start(self) -> int:
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == config.START1 and self.buffer[i + 1] == config.START2:
                return i
        return -1


def try_extract_ascii_text(payload: bytes) -> Optional[str]:
    cleaned = payload.replace(b'\x00', b'')
    if not cleaned:
        return None
    try:
        text = cleaned.decode('utf-8', errors='ignore').strip()
    except Exception:
        return None
    if not text:
        return None
    printable = sum(1 for ch in text if ch.isprintable() or ch in '\r\n\t')
    if printable / max(len(text), 1) < 0.85:
        return None
    return text


def publish_raw(mqtt_pub, payload: bytes, note: str = 'raw-frame', decoded: Optional[dict] = None) -> None:
    if not config.FORWARD_RAW_BASE64:
        return
    msg = {'ts': utc_now(), 'kind': note, 'payload_b64': base64.b64encode(payload).decode('ascii'), 'payload_hex': payload.hex()}
    if decoded is not None:
        msg['decoded'] = decoded
    mqtt_pub.publish_json(config.MQTT_RAW_TOPIC, msg)


def decode_from_radio(payload: bytes):
    if mesh_pb2 is None:
        return None
    msg = mesh_pb2.FromRadio()
    try:
        msg.ParseFromString(payload)
        return msg
    except DecodeError:
        return None


def packet_meta(packet, portnum: int, raw_payload: bytes) -> Dict:
    return {
        'ts': utc_now(), 'from': getattr(packet, 'from', 0), 'to': getattr(packet, 'to', 0), 'id': getattr(packet, 'id', 0),
        'channel': getattr(packet, 'channel', 0), 'rx_time': getattr(packet, 'rx_time', 0), 'rx_snr': getattr(packet, 'rx_snr', 0),
        'hop_limit': getattr(packet, 'hop_limit', 0), 'want_ack': getattr(packet, 'want_ack', False), 'priority': int(getattr(packet, 'priority', 0)),
        'portnum': portnum, 'payload_b64': base64.b64encode(raw_payload).decode('ascii'),
    }


def maybe_extract_text_message(from_radio) -> Optional[Dict]:
    if from_radio is None or not from_radio.HasField('packet'):
        return None
    packet = from_radio.packet
    if not packet.HasField('decoded'):
        return None
    decoded = packet.decoded
    portnum = int(decoded.portnum)
    raw_payload = bytes(decoded.payload)
    if portnum not in {config.TEXT_MESSAGE_APP, config.TEXT_MESSAGE_COMPRESSED_APP, config.DETECTION_SENSOR_APP, config.ALERT_APP}:
        return None
    text = try_extract_ascii_text(raw_payload) or base64.b64encode(raw_payload).decode('ascii')
    msg = packet_meta(packet, portnum, raw_payload)
    msg['text'] = text
    return msg


def decode_vision_message(raw_payload: bytes) -> Optional[Dict]:
    if mesh_pb2 is None:
        return None
    try:
        vision = mesh_pb2.VisionMessage()
        vision.ParseFromString(raw_payload)
    except DecodeError:
        return None
    msg = MessageToDict(vision, preserving_proto_field_name=True)
    if vision.HasField('new_detection'):
        nd = vision.new_detection
        xywh = int(nd.xywh_packed)
        msg.update({'type': 'new_detection', 'x': xywh & 0xFF, 'y': (xywh >> 8) & 0xFF, 'w': (xywh >> 16) & 0xFF, 'h': (xywh >> 24) & 0xFF,
                    'conf': int(nd.conf), 'class_id': int(nd.class_id), 'embed_crc8': int(nd.embed_crc8),
                    'embedding_b64': base64.b64encode(bytes(nd.embedding)).decode('ascii') if nd.embedding else ''})
    elif vision.HasField('update_detection'):
        ud = vision.update_detection
        xywh = int(ud.xywh_packed)
        msg.update({'type': 'update_detection', 'x': xywh & 0xFF, 'y': (xywh >> 8) & 0xFF, 'w': (xywh >> 16) & 0xFF, 'h': (xywh >> 24) & 0xFF,
                    'conf': int(ud.conf), 'age': int(ud.age)})
    else:
        msg['type'] = 'unknown'
    return msg


def maybe_extract_vision_message(from_radio) -> Optional[Dict]:
    if from_radio is None or not from_radio.HasField('packet'):
        return None
    packet = from_radio.packet
    if not packet.HasField('decoded'):
        return None
    decoded = packet.decoded
    portnum = int(decoded.portnum)
    raw_payload = bytes(decoded.payload)
    if portnum != config.PRIVATE_APP:
        return None
    vision = decode_vision_message(raw_payload)
    if vision is None:
        return None
    msg = packet_meta(packet, portnum, raw_payload)
    msg.update(vision)
    return msg


def handle_frame(mqtt_pub, payload: bytes) -> None:
    from_radio = decode_from_radio(payload)
    if from_radio is not None:
        mark_rx()
        vision_msg = maybe_extract_vision_message(from_radio)
        if vision_msg is not None:
            mqtt_pub.publish_json(config.MQTT_VISION_TOPIC, vision_msg)
            logging.info('[mesh:vision] from=%s track=%s type=%s', vision_msg.get('from'), vision_msg.get('track_id'), vision_msg.get('type'))
            return
        text_msg = maybe_extract_text_message(from_radio)
        if text_msg is not None:
            mqtt_pub.publish_json(config.MQTT_TEXT_TOPIC, text_msg)
            logging.info('[mesh:text] from=%s to=%s ch=%s port=%s text=%s', text_msg['from'], text_msg['to'], text_msg['channel'], text_msg['portnum'], text_msg['text'])
            return
        publish_raw(mqtt_pub, payload, note='from-radio', decoded=MessageToDict(from_radio, preserving_proto_field_name=True))
        return
    text = try_extract_ascii_text(payload)
    if text:
        mqtt_pub.publish_json(config.MQTT_TEXT_TOPIC, {'ts': utc_now(), 'text': text, 'unframed': True})
        logging.info('[mesh:text:fallback] %s', text)
        return
    publish_raw(mqtt_pub, payload)
