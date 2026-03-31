import base64
import json
import logging
import os
import platform
import shutil
import socket
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Dict, Optional, Tuple

import paho.mqtt.client as mqtt
from dotenv import load_dotenv
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError
from pymavlink import mavutil

try:
    from meshtastic import mesh_pb2
except ImportError:
    mesh_pb2 = None

SENT_RETENTION_DAYS = 7
MAVLINK_RECENT_WINDOW_SECONDS = 600
TELEMETRY_INTERVAL_SECONDS = 60

load_dotenv()

MAVLINK_TCP_HOST = os.getenv("MAVLINK_TCP_HOST", "127.0.0.1")
MAVLINK_TCP_PORT = int(os.getenv("MAVLINK_TCP_PORT", "5760"))
MQTT_URL = os.getenv("MQTT_URL", "mqtt://localhost:1883")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_VISION_TOPIC = os.getenv("MQTT_VISION_TOPIC", "sentai/vision")
MQTT_TEXT_TOPIC = os.getenv("MQTT_TEXT_TOPIC", "sentai/text")
MQTT_RAW_TOPIC = os.getenv("MQTT_RAW_TOPIC", "sentai/raw")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "sentai/status")
MQTT_AVAILABILITY_TOPIC = os.getenv("MQTT_AVAILABILITY_TOPIC", "sentai/availability")
SPOOL_ROOT = Path(os.getenv("SPOOL_ROOT", str(Path(__file__).resolve().parent / "spool")))
PENDING_DIR = SPOOL_ROOT / "pending"
SENT_DIR = SPOOL_ROOT / "Sent"
FORWARD_RAW_BASE64 = str(os.getenv("FORWARD_RAW_BASE64", "true")).lower() in {"1", "true", "yes", "on"}

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

LAST_LINK_RX_TS = 0.0
LAST_LINK_RX_LOCK = Lock()
LAST_TELEMETRY_TS = 0.0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def ensure_dirs() -> None:
    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    SENT_DIR.mkdir(parents=True, exist_ok=True)


def cleanup_sent_dir() -> None:
    now = time.time()
    max_age = SENT_RETENTION_DAYS * 24 * 3600
    for path in SENT_DIR.glob("*.json"):
        try:
            if now - path.stat().st_mtime > max_age:
                path.unlink(missing_ok=True)
        except Exception as exc:
            logging.warning("[spool] cleanup failed for %s: %s", path, exc)


def write_spool(topic: str, payload: dict) -> Path:
    ensure_dirs()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    path = PENDING_DIR / f"{stamp}_{uuid.uuid4().hex}.json"
    path.write_text(json.dumps({"topic": topic, "payload": payload, "queued_at": utc_now()}, ensure_ascii=False), encoding="utf-8")
    return path


def mark_sent(path: Path) -> None:
    ensure_dirs()
    shutil.move(str(path), str(SENT_DIR / path.name))


def parse_mqtt_url(url: str) -> Tuple[str, int]:
    if "://" in url:
        _, rest = url.split("://", 1)
    else:
        rest = url
    if "/" in rest:
        rest = rest.split("/", 1)[0]
    if ":" in rest:
        host, port = rest.rsplit(":", 1)
        return host, int(port)
    return rest, 1883


def mark_link_rx() -> None:
    global LAST_LINK_RX_TS
    with LAST_LINK_RX_LOCK:
        LAST_LINK_RX_TS = time.time()


def has_recent_link_rx() -> bool:
    with LAST_LINK_RX_LOCK:
        return (time.time() - LAST_LINK_RX_TS) <= MAVLINK_RECENT_WINDOW_SECONDS if LAST_LINK_RX_TS else False


def get_ip_addresses() -> list:
    try:
        return [x for x in os.popen("hostname -I").read().strip().split() if x]
    except Exception:
        return []


def get_meminfo() -> dict:
    result = {}
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                result[k.strip()] = v.strip()
    except Exception:
        pass
    return result


def get_uptime_seconds() -> float:
    try:
        return float(Path('/proc/uptime').read_text().split()[0])
    except Exception:
        return 0.0


def get_disk_usage() -> dict:
    st = shutil.disk_usage('/')
    return {'total_bytes': st.total, 'used_bytes': st.used, 'free_bytes': st.free}


def get_system_metrics() -> dict:
    meminfo = get_meminfo()
    total_kb = int(meminfo.get('MemTotal', '0 kB').split()[0]) if meminfo.get('MemTotal') else 0
    avail_kb = int(meminfo.get('MemAvailable', '0 kB').split()[0]) if meminfo.get('MemAvailable') else 0
    used_kb = max(total_kb - avail_kb, 0)
    load1, load5, load15 = os.getloadavg()
    return {
        'hostname': platform.node(),
        'ip_addresses': get_ip_addresses(),
        'os': platform.platform(),
        'python': platform.python_version(),
        'cpu_count': os.cpu_count(),
        'loadavg': {'1m': load1, '5m': load5, '15m': load15},
        'memory': {'total_kb': total_kb, 'available_kb': avail_kb, 'used_kb': used_kb},
        'disk_root': get_disk_usage(),
        'uptime_seconds': get_uptime_seconds(),
    }


def build_telemetry() -> dict:
    now = datetime.now(timezone.utc)
    return {
        'ts': now.isoformat(),
        'date': now.strftime('%Y-%m-%d'),
        'time': now.strftime('%H:%M:%S'),
        'kind': 'availability',
        'mavlink_tcp_host': MAVLINK_TCP_HOST,
        'mavlink_tcp_port': MAVLINK_TCP_PORT,
        'mavlink_message_received_last_10m': has_recent_link_rx(),
        'system': get_system_metrics(),
    }


class MqttPublisher:
    def __init__(self) -> None:
        self.connected = False
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if MQTT_USERNAME:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD or None)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

    def connect(self) -> None:
        host, port = parse_mqtt_url(MQTT_URL)
        self.client.connect_async(host, port, keepalive=60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.connected = True
        self.publish_json(MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'connected', 'mavlink_tcp_host': MAVLINK_TCP_HOST, 'mavlink_tcp_port': MAVLINK_TCP_PORT})
        self.flush_pending()

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        logging.warning('[mqtt] disconnected: %s', reason_code)

    def publish_json(self, topic: str, payload: dict) -> None:
        if not self.connected:
            write_spool(topic, payload)
            return
        info = self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
        info.wait_for_publish(timeout=5)
        if info.rc != mqtt.MQTT_ERR_SUCCESS or not info.is_published():
            write_spool(topic, payload)

    def flush_pending(self) -> None:
        ensure_dirs()
        cleanup_sent_dir()
        for path in sorted(PENDING_DIR.glob('*.json')):
            try:
                env = json.loads(path.read_text(encoding='utf-8'))
                info = self.client.publish(env['topic'], json.dumps(env['payload'], ensure_ascii=False), qos=1, retain=False)
                info.wait_for_publish(timeout=5)
                if info.rc == mqtt.MQTT_ERR_SUCCESS and info.is_published():
                    mark_sent(path)
                else:
                    break
            except Exception:
                break


class StatusTextAssembler:
    def __init__(self):
        self.buffers = {}

    def push(self, msg) -> Optional[Tuple[str, dict]]:
        text = bytes(msg.text).decode('utf-8', errors='ignore').rstrip('\x00') if not isinstance(msg.text, str) else msg.text.rstrip('\x00')
        sev = int(msg.severity)
        msg_id = int(getattr(msg, 'id', 0))
        chunk_seq = int(getattr(msg, 'chunk_seq', 0))

        if msg_id == 0:
            return ('text', {'ts': utc_now(), 'severity': sev, 'text': text})

        entry = self.buffers.setdefault(msg_id, {'chunks': {}, 'severity': sev, 'first_ts': time.time()})
        entry['chunks'][chunk_seq] = text

        complete = False
        if len(text) < 50:
            complete = True
        else:
            expected = sorted(entry['chunks'].keys())
            complete = expected == list(range(0, max(expected) + 1)) and any(len(entry['chunks'][k]) < 50 for k in expected)

        if not complete:
            return None

        parts = []
        for idx in sorted(entry['chunks']):
            parts.append(entry['chunks'][idx])
        del self.buffers[msg_id]
        b64 = ''.join(parts)
        return ('chunked', {'id': msg_id, 'severity': sev, 'base64': b64})

    def cleanup(self, max_age_seconds: int = 120) -> None:
        now = time.time()
        stale = [k for k, v in self.buffers.items() if now - v['first_ts'] > max_age_seconds]
        for k in stale:
            del self.buffers[k]


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
        msg['type'] = 'new_detection'
        msg['x'] = xywh & 0xFF
        msg['y'] = (xywh >> 8) & 0xFF
        msg['w'] = (xywh >> 16) & 0xFF
        msg['h'] = (xywh >> 24) & 0xFF
        msg['conf'] = int(nd.conf)
        msg['class_id'] = int(nd.class_id)
        msg['embed_crc8'] = int(nd.embed_crc8)
        msg['embedding_b64'] = base64.b64encode(bytes(nd.embedding)).decode('ascii') if nd.embedding else ''
    elif vision.HasField('update_detection'):
        ud = vision.update_detection
        xywh = int(ud.xywh_packed)
        msg['type'] = 'update_detection'
        msg['x'] = xywh & 0xFF
        msg['y'] = (xywh >> 8) & 0xFF
        msg['w'] = (xywh >> 16) & 0xFF
        msg['h'] = (xywh >> 24) & 0xFF
        msg['conf'] = int(ud.conf)
        msg['age'] = int(ud.age)
    else:
        msg['type'] = 'unknown'
    return msg


def maybe_publish_telemetry(mqtt_pub: MqttPublisher) -> None:
    global LAST_TELEMETRY_TS
    now = time.time()
    if now - LAST_TELEMETRY_TS < TELEMETRY_INTERVAL_SECONDS:
        return
    LAST_TELEMETRY_TS = now
    mqtt_pub.publish_json(MQTT_AVAILABILITY_TOPIC, build_telemetry())


def main() -> int:
    ensure_dirs()
    cleanup_sent_dir()

    mqtt_pub = MqttPublisher()
    mqtt_pub.connect()
    mqtt_pub.publish_json(MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'starting', 'mavlink_tcp_host': MAVLINK_TCP_HOST, 'mavlink_tcp_port': MAVLINK_TCP_PORT, 'protobuf_enabled': mesh_pb2 is not None})

    assembler = StatusTextAssembler()
    endpoint = f'tcp:{MAVLINK_TCP_HOST}:{MAVLINK_TCP_PORT}'

    while True:
        try:
            conn = mavutil.mavlink_connection(endpoint, source_system=250, source_component=1, autoreconnect=True)
            mqtt_pub.publish_json(MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'mavlink-connected', 'endpoint': endpoint})
            logging.info('[link] connected to %s', endpoint)

            while True:
                if mqtt_pub.connected:
                    mqtt_pub.flush_pending()
                maybe_publish_telemetry(mqtt_pub)
                assembler.cleanup()

                msg = conn.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue

                mark_link_rx()
                msg_type = msg.get_type()

                if msg_type == 'STATUSTEXT':
                    assembled = assembler.push(msg)
                    if not assembled:
                        continue
                    kind, data = assembled
                    if kind == 'text':
                        mqtt_pub.publish_json(MQTT_TEXT_TOPIC, data)
                        continue

                    vision = decode_vision_message_from_b64(data['base64'])
                    if vision is not None:
                        out = {'ts': utc_now(), 'mavlink_msg_id': data['id'], 'severity': data['severity'], **vision}
                        mqtt_pub.publish_json(MQTT_VISION_TOPIC, out)
                    elif FORWARD_RAW_BASE64:
                        mqtt_pub.publish_json(MQTT_RAW_TOPIC, {'ts': utc_now(), 'kind': 'statustext-chunked', 'mavlink_msg_id': data['id'], 'severity': data['severity'], 'payload_b64': data['base64']})
                else:
                    if FORWARD_RAW_BASE64:
                        mqtt_pub.publish_json(MQTT_RAW_TOPIC, {'ts': utc_now(), 'kind': 'mavlink', 'mavlink_type': msg_type, 'message': msg.to_dict()})
        except KeyboardInterrupt:
            logging.info('stopped by user')
            return 0
        except (socket.error, OSError) as exc:
            logging.warning('[link] connection error: %s', exc)
            mqtt_pub.publish_json(MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'mavlink-disconnected', 'error': str(exc), 'endpoint': endpoint})
            time.sleep(2)
        except Exception as exc:
            logging.exception('linkproxy failed: %s', exc)
            mqtt_pub.publish_json(MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'error', 'error': str(exc)})
            time.sleep(2)


if __name__ == '__main__':
    sys.exit(main())
