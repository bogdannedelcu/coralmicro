import base64
import logging
from typing import Dict, Optional

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError
from pubsub import pub

from commonproxy import utc_now

from . import config
from .common import recent_rx

try:
    from commonproxy.protos import visionmesh_pb2
except ImportError:
    visionmesh_pb2 = None


class MeshSdkReceiver:
    def __init__(self, mqtt_pub) -> None:
        self.mqtt_pub = mqtt_pub

    def subscribe(self) -> None:
        pub.subscribe(self._on_text, 'meshtastic.receive.text')
        pub.subscribe(self._on_data, 'meshtastic.receive.data')
        pub.subscribe(self._on_unknown, 'meshtastic.receive')
        pub.subscribe(self._on_connection_lost, 'meshtastic.connection.lost')

    def _on_text(self, packet, interface) -> None:
        recent_rx.mark()
        decoded = packet.get('decoded', {})
        payload = decoded.get('payload', b'')
        if isinstance(payload, str):
            text = payload
        else:
            try:
                text = bytes(payload).decode('utf-8', errors='ignore').strip()
            except Exception:
                text = ''
        self.mqtt_pub.publish_json(config.MQTT_TEXT_TOPIC, {
            'ts': utc_now(),
            'from': packet.get('from', 0),
            'to': packet.get('to', 0),
            'id': packet.get('id', 0),
            'channel': packet.get('channel', 0),
            'portnum': decoded.get('portnum', 'TEXT_MESSAGE_APP'),
            'text': text,
        })

    def _on_data(self, packet, interface) -> None:
        recent_rx.mark()
        decoded = packet.get('decoded', {})
        portnum = decoded.get('portnum')
        payload = decoded.get('payload', b'')
        raw_payload = payload.encode() if isinstance(payload, str) else bytes(payload)

        if portnum == 'PRIVATE_APP':
            vision = decode_vision_message(raw_payload)
            if vision is not None:
                msg = {
                    'ts': utc_now(),
                    'from': packet.get('from', 0),
                    'to': packet.get('to', 0),
                    'id': packet.get('id', 0),
                    'channel': packet.get('channel', 0),
                    'portnum': portnum,
                    'payload_b64': base64.b64encode(raw_payload).decode('ascii'),
                    **vision,
                }
                if not msg.get('node_id'):
                    msg['node_id'] = packet.get('from', 0)
                self.mqtt_pub.publish_json(config.MQTT_VISION_TOPIC, msg)
                logging.info('[mesh:vision] from=%s track=%s type=%s', msg.get('from'), msg.get('track_id'), msg.get('type'))
                return

        if config.FORWARD_RAW_BASE64:
            self.mqtt_pub.publish_json(config.MQTT_RAW_TOPIC, {
                'ts': utc_now(),
                'kind': 'from-radio',
                'packet': sanitize_packet(packet),
            })

    def _on_unknown(self, packet, interface) -> None:
        recent_rx.mark()

    def _on_connection_lost(self, interface) -> None:
        self.mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'connection-lost'})


def sanitize_packet(packet: dict) -> dict:
    out = dict(packet)
    raw = out.get('raw')
    if raw is not None:
        out['raw'] = str(type(raw).__name__)
    decoded = out.get('decoded')
    if isinstance(decoded, dict):
        d = dict(decoded)
        payload = d.get('payload')
        if isinstance(payload, (bytes, bytearray)):
            d['payload_b64'] = base64.b64encode(bytes(payload)).decode('ascii')
            del d['payload']
        out['decoded'] = d
    return out


def decode_vision_message(raw_payload: bytes) -> Optional[Dict]:
    if visionmesh_pb2 is None:
        return None
    try:
        vision = visionmesh_pb2.VisionMessage()
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
