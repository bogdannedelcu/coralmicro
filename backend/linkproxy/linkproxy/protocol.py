import base64
import json
import logging
import time
from typing import Dict, Optional, Tuple

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError

from . import config
from .common import mark_rx, utc_now

try:
    from meshtastic import mesh_pb2
except ImportError:
    mesh_pb2 = None


class StatusTextAssembler:
    def __init__(self) -> None:
        self.buffers: Dict[int, Dict] = {}

    def push(self, msg) -> Optional[Tuple[str, dict]]:
        text = self._decode_text_field(msg)
        severity = int(msg.severity)
        msg_id = int(getattr(msg, 'id', 0))
        chunk_seq = int(getattr(msg, 'chunk_seq', 0))

        if msg_id == 0:
            return 'text', {'ts': utc_now(), 'severity': severity, 'text': text}

        entry = self.buffers.setdefault(msg_id, {'chunks': {}, 'severity': severity, 'first_ts': time.time()})
        entry['chunks'][chunk_seq] = text

        if len(text) >= config.STATUSTEXT_CHUNK_SIZE:
            return None

        ordered_keys = sorted(entry['chunks'])
        if ordered_keys != list(range(0, max(ordered_keys) + 1)):
            return None

        joined = ''.join(entry['chunks'][idx] for idx in ordered_keys)
        del self.buffers[msg_id]
        return 'chunked', {'id': msg_id, 'severity': severity, 'base64': joined}

    def cleanup(self, max_age_seconds: int = 120) -> None:
        now = time.time()
        stale = [k for k, v in self.buffers.items() if now - v['first_ts'] > max_age_seconds]
        for key in stale:
            del self.buffers[key]

    @staticmethod
    def _decode_text_field(msg) -> str:
        field = msg.text
        if isinstance(field, str):
            return field.rstrip('\x00')
        try:
            return bytes(field).decode('utf-8', errors='ignore').rstrip('\x00')
        except Exception:
            return str(field).rstrip('\x00')


def decode_vision_message_from_b64(b64_text: str) -> Optional[Dict]:
    if mesh_pb2 is None:
        return None
    try:
        raw = base64.b64decode(b64_text)
        vision = mesh_pb2.VisionMessage()
        vision.ParseFromString(raw)
    except (DecodeError, ValueError, base64.binascii.Error):
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


def handle_statustext(mqtt_pub, assembler: StatusTextAssembler, msg) -> None:
    assembled = assembler.push(msg)
    if not assembled:
        return
    kind, data = assembled
    if kind == 'text':
        mqtt_pub.publish_json(config.MQTT_TEXT_TOPIC, data)
        return
    vision = decode_vision_message_from_b64(data['base64'])
    if vision is not None:
        out = {'ts': utc_now(), 'mavlink_msg_id': data['id'], 'severity': data['severity'], **vision}
        mqtt_pub.publish_json(config.MQTT_VISION_TOPIC, out)
        logging.info('[link:vision] msg_id=%s type=%s track=%s', data['id'], out.get('type'), out.get('track_id'))
    elif config.FORWARD_RAW_BASE64:
        mqtt_pub.publish_json(config.MQTT_RAW_TOPIC, {'ts': utc_now(), 'kind': 'statustext-chunked', 'mavlink_msg_id': data['id'], 'severity': data['severity'], 'payload_b64': data['base64']})


def run_selftest() -> int:
    if mesh_pb2 is None:
        print('SELFTEST: meshtastic package unavailable')
        return 1
    vision = mesh_pb2.VisionMessage()
    vision.app_version = 1
    vision.sensor_id = 123
    vision.track_id = 456
    vision.alarm_type = 2
    vision.timestamp_utc = 1774955000
    vision.seq = 42
    vision.new_detection.xywh_packed = (10) | (20 << 8) | (30 << 16) | (40 << 24)
    vision.new_detection.conf = 92
    vision.new_detection.class_id = 7
    vision.new_detection.embed_crc8 = 55
    vision.new_detection.embedding = b'\x01\x02\x03\x04'
    b64 = base64.b64encode(vision.SerializeToString()).decode('ascii')
    chunks = [b64[i:i + config.STATUSTEXT_CHUNK_SIZE] for i in range(0, len(b64), config.STATUSTEXT_CHUNK_SIZE)]

    class MockMsg:
        def __init__(self, text, severity, msg_id, chunk_seq):
            self.text = text
            self.severity = severity
            self.id = msg_id
            self.chunk_seq = chunk_seq

    assembler = StatusTextAssembler()
    result = None
    for i, chunk in enumerate(chunks):
        result = assembler.push(MockMsg(chunk, 6, 77, i))
    if not result or result[0] != 'chunked':
        print('SELFTEST: FAILED assembling chunks')
        return 2
    decoded = decode_vision_message_from_b64(result[1]['base64'])
    if not decoded:
        print('SELFTEST: FAILED decoding vision message')
        return 3
    print(json.dumps(decoded, ensure_ascii=False))
    return 0


def handle_message(mqtt_pub, assembler: StatusTextAssembler, msg) -> None:
    mark_rx()
    if msg.get_type() == 'STATUSTEXT':
        handle_statustext(mqtt_pub, assembler, msg)
    elif config.FORWARD_RAW_BASE64:
        mqtt_pub.publish_json(config.MQTT_RAW_TOPIC, {'ts': utc_now(), 'kind': 'mavlink', 'mavlink_type': msg.get_type(), 'message': msg.to_dict()})
