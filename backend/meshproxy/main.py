import base64
import platform
import json
import logging
import os
import shutil
import struct
import sys
import time
import uuid
from threading import Lock
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional, Tuple

import paho.mqtt.client as mqtt
import serial
from serial.tools import list_ports
from dotenv import load_dotenv
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError

try:
    from meshtastic import mesh_pb2, portnums_pb2
except ImportError:
    mesh_pb2 = None
    portnums_pb2 = None


START1 = 0x94
START2 = 0xC3
TEXT_MESSAGE_APP = 1
TEXT_MESSAGE_COMPRESSED_APP = 7
DETECTION_SENSOR_APP = 10
ALERT_APP = 11
PRIVATE_APP = 256
SENT_RETENTION_DAYS = 7
MESH_RECENT_WINDOW_SECONDS = 600
TELEMETRY_INTERVAL_SECONDS = 60


load_dotenv()

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "38400"))
MQTT_URL = os.getenv("MQTT_URL", "mqtt://localhost:1883")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_TOPIC_PREFIX = os.getenv("MQTT_TOPIC_PREFIX", "sentai")
MQTT_VISION_TOPIC = os.getenv("MQTT_VISION_TOPIC", "sentai/vision")
MQTT_TEXT_TOPIC = os.getenv("MQTT_TEXT_TOPIC", "sentai/text")
MQTT_RAW_TOPIC = os.getenv("MQTT_RAW_TOPIC", "sentai/raw")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "sentai/status")
MQTT_AVAILABILITY_TOPIC = os.getenv("MQTT_AVAILABILITY_TOPIC", "sentai/availability")
FORWARD_RAW_BASE64 = str(os.getenv("FORWARD_RAW_BASE64", "true")).lower() in {"1", "true", "yes", "on"}
SPOOL_ROOT = Path(os.getenv("SPOOL_ROOT", str(Path(__file__).resolve().parent / "spool")))
PENDING_DIR = SPOOL_ROOT / "pending"
SENT_DIR = SPOOL_ROOT / "Sent"


logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


LAST_MESH_RX_TS = 0.0
LAST_MESH_RX_LOCK = Lock()
LAST_TELEMETRY_TS = 0.0


def mark_mesh_rx() -> None:
    global LAST_MESH_RX_TS
    with LAST_MESH_RX_LOCK:
        LAST_MESH_RX_TS = time.time()


def has_recent_mesh_rx() -> bool:
    with LAST_MESH_RX_LOCK:
        return (time.time() - LAST_MESH_RX_TS) <= MESH_RECENT_WINDOW_SECONDS if LAST_MESH_RX_TS else False


def list_serial_ports() -> list:
    ports = []
    for p in list_ports.comports():
        ports.append({
            "device": p.device,
            "description": p.description,
            "hwid": p.hwid,
        })
    return ports


def get_meminfo() -> dict:
    result = {}
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if ':' not in line:
                continue
            key, val = line.split(':', 1)
            result[key.strip()] = val.strip()
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
    return {
        'total_bytes': st.total,
        'used_bytes': st.used,
        'free_bytes': st.free,
    }


def get_ip_addresses() -> list:
    ips = []
    try:
        for line in os.popen("hostname -I").read().strip().split():
            if line not in ips:
                ips.append(line)
    except Exception:
        pass
    return ips


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
        'loadavg': {
            '1m': load1,
            '5m': load5,
            '15m': load15,
        },
        'memory': {
            'total_kb': total_kb,
            'available_kb': avail_kb,
            'used_kb': used_kb,
        },
        'disk_root': get_disk_usage(),
        'uptime_seconds': get_uptime_seconds(),
    }


def build_telemetry() -> dict:
    return {
        "ts": utc_now(),
        "kind": "availability",
        "serial_configured_port": SERIAL_PORT,
        "serial_baud": SERIAL_BAUD,
        "serial_ports": list_serial_ports(),
        "mesh_message_received_last_10m": has_recent_mesh_rx(),
        "system": get_system_metrics(),
    }


def maybe_publish_telemetry(mqtt_pub) -> None:
    global LAST_TELEMETRY_TS
    now = time.time()
    if now - LAST_TELEMETRY_TS < TELEMETRY_INTERVAL_SECONDS:
        return
    LAST_TELEMETRY_TS = now
    mqtt_pub.publish_json(MQTT_AVAILABILITY_TOPIC, build_telemetry())


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


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


def ensure_dirs() -> None:
    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    SENT_DIR.mkdir(parents=True, exist_ok=True)


def cleanup_sent_dir() -> None:
    now = time.time()
    max_age = SENT_RETENTION_DAYS * 24 * 3600
    for path in SENT_DIR.glob("*.json"):
        try:
            age = now - path.stat().st_mtime
            if age > max_age:
                path.unlink(missing_ok=True)
        except Exception as exc:
            logging.warning("[spool] cleanup failed for %s: %s", path, exc)


def write_spool(topic: str, payload: dict) -> Path:
    ensure_dirs()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    fname = f"{stamp}_{uuid.uuid4().hex}.json"
    path = PENDING_DIR / fname
    envelope = {
        "topic": topic,
        "payload": payload,
        "queued_at": utc_now(),
    }
    path.write_text(json.dumps(envelope, ensure_ascii=False), encoding="utf-8")
    return path


def mark_sent(spool_path: Path) -> None:
    ensure_dirs()
    target = SENT_DIR / spool_path.name
    shutil.move(str(spool_path), str(target))


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
        logging.info("[mqtt] connected to %s", MQTT_URL)
        self.publish_json(
            MQTT_STATUS_TOPIC,
            {
                "ts": utc_now(),
                "state": "connected",
                "serial_port": SERIAL_PORT,
                "serial_baud": SERIAL_BAUD,
            },
        )
        self.flush_pending()

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        logging.warning("[mqtt] disconnected: %s", reason_code)

    def publish_json(self, topic: str, payload: dict) -> None:
        if not self.connected:
            path = write_spool(topic, payload)
            logging.info("[spool] queued %s -> %s", topic, path.name)
            return

        info = self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
        info.wait_for_publish(timeout=5)
        if info.rc != mqtt.MQTT_ERR_SUCCESS or not info.is_published():
            path = write_spool(topic, payload)
            logging.warning("[mqtt] publish failed, queued %s -> %s", topic, path.name)

    def flush_pending(self) -> None:
        ensure_dirs()
        cleanup_sent_dir()
        pending = sorted(PENDING_DIR.glob("*.json"))
        if pending:
            logging.info("[spool] flushing %d queued messages", len(pending))
        for path in pending:
            try:
                envelope = json.loads(path.read_text(encoding="utf-8"))
                topic = envelope["topic"]
                payload = envelope["payload"]
                info = self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
                info.wait_for_publish(timeout=5)
                if info.rc == mqtt.MQTT_ERR_SUCCESS and info.is_published():
                    mark_sent(path)
                else:
                    logging.warning("[spool] publish retry failed for %s", path.name)
                    break
            except Exception as exc:
                logging.warning("[spool] flush failed for %s: %s", path.name, exc)
                break


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
            length = struct.unpack(">H", self.buffer[2:4])[0]
            frame_len = 4 + length
            if len(self.buffer) < frame_len:
                break
            payload = bytes(self.buffer[4:frame_len])
            del self.buffer[:frame_len]
            frames.append(payload)
        return frames

    def _find_start(self) -> int:
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == START1 and self.buffer[i + 1] == START2:
                return i
        return -1


def try_extract_ascii_text(payload: bytes) -> Optional[str]:
    cleaned = payload.replace(b"\x00", b"")
    if not cleaned:
        return None
    try:
        text = cleaned.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None
    if not text:
        return None
    printable = sum(1 for ch in text if ch.isprintable() or ch in "\r\n\t")
    if printable / max(len(text), 1) < 0.85:
        return None
    return text


def publish_raw(mqtt_pub: MqttPublisher, payload: bytes, note: str = "raw-frame", decoded: Optional[dict] = None) -> None:
    if not FORWARD_RAW_BASE64:
        return
    msg = {
        "ts": utc_now(),
        "kind": note,
        "payload_b64": base64.b64encode(payload).decode("ascii"),
        "payload_hex": payload.hex(),
    }
    if decoded is not None:
        msg["decoded"] = decoded
    mqtt_pub.publish_json(MQTT_RAW_TOPIC, msg)


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
        "ts": utc_now(),
        "from": getattr(packet, "from", 0),
        "to": getattr(packet, "to", 0),
        "id": getattr(packet, "id", 0),
        "channel": getattr(packet, "channel", 0),
        "rx_time": getattr(packet, "rx_time", 0),
        "rx_snr": getattr(packet, "rx_snr", 0),
        "hop_limit": getattr(packet, "hop_limit", 0),
        "want_ack": getattr(packet, "want_ack", False),
        "priority": int(getattr(packet, "priority", 0)),
        "portnum": portnum,
        "payload_b64": base64.b64encode(raw_payload).decode("ascii"),
    }


def maybe_extract_text_message(from_radio) -> Optional[Dict]:
    if from_radio is None or not from_radio.HasField("packet"):
        return None
    packet = from_radio.packet
    if not packet.HasField("decoded"):
        return None
    decoded = packet.decoded
    portnum = int(decoded.portnum)
    raw_payload = bytes(decoded.payload)
    if portnum not in {TEXT_MESSAGE_APP, TEXT_MESSAGE_COMPRESSED_APP, DETECTION_SENSOR_APP, ALERT_APP}:
        return None
    text = try_extract_ascii_text(raw_payload)
    if text is None:
        text = base64.b64encode(raw_payload).decode("ascii")
    msg = packet_meta(packet, portnum, raw_payload)
    msg["text"] = text
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
    if vision.HasField("new_detection"):
        nd = vision.new_detection
        xywh = int(nd.xywh_packed)
        msg["type"] = "new_detection"
        msg["x"] = xywh & 0xFF
        msg["y"] = (xywh >> 8) & 0xFF
        msg["w"] = (xywh >> 16) & 0xFF
        msg["h"] = (xywh >> 24) & 0xFF
        msg["conf"] = int(nd.conf)
        msg["class_id"] = int(nd.class_id)
        msg["embed_crc8"] = int(nd.embed_crc8)
        msg["embedding_b64"] = base64.b64encode(bytes(nd.embedding)).decode("ascii") if nd.embedding else ""
    elif vision.HasField("update_detection"):
        ud = vision.update_detection
        xywh = int(ud.xywh_packed)
        msg["type"] = "update_detection"
        msg["x"] = xywh & 0xFF
        msg["y"] = (xywh >> 8) & 0xFF
        msg["w"] = (xywh >> 16) & 0xFF
        msg["h"] = (xywh >> 24) & 0xFF
        msg["conf"] = int(ud.conf)
        msg["age"] = int(ud.age)
    else:
        msg["type"] = "unknown"
    return msg


def maybe_extract_vision_message(from_radio) -> Optional[Dict]:
    if from_radio is None or not from_radio.HasField("packet"):
        return None
    packet = from_radio.packet
    if not packet.HasField("decoded"):
        return None
    decoded = packet.decoded
    portnum = int(decoded.portnum)
    raw_payload = bytes(decoded.payload)
    if portnum != PRIVATE_APP:
        return None
    vision = decode_vision_message(raw_payload)
    if vision is None:
        return None
    msg = packet_meta(packet, portnum, raw_payload)
    msg.update(vision)
    return msg


def handle_frame(mqtt_pub: MqttPublisher, payload: bytes) -> None:
    from_radio = decode_from_radio(payload)
    if from_radio is not None:
        mark_mesh_rx()
        vision_msg = maybe_extract_vision_message(from_radio)
        if vision_msg is not None:
            mqtt_pub.publish_json(MQTT_VISION_TOPIC, vision_msg)
            logging.info("[mesh:vision] from=%s track=%s type=%s", vision_msg.get("from"), vision_msg.get("track_id"), vision_msg.get("type"))
            return
        text_msg = maybe_extract_text_message(from_radio)
        if text_msg is not None:
            mqtt_pub.publish_json(MQTT_TEXT_TOPIC, text_msg)
            logging.info("[mesh:text] from=%s to=%s ch=%s port=%s text=%s", text_msg["from"], text_msg["to"], text_msg["channel"], text_msg["portnum"], text_msg["text"])
            return
        publish_raw(mqtt_pub, payload, note="from-radio", decoded=MessageToDict(from_radio, preserving_proto_field_name=True))
        return

    text = try_extract_ascii_text(payload)
    if text:
        mqtt_pub.publish_json(MQTT_TEXT_TOPIC, {"ts": utc_now(), "text": text, "unframed": True})
        logging.info("[mesh:text:fallback] %s", text)
        return
    publish_raw(mqtt_pub, payload)


def main() -> int:
    ensure_dirs()
    cleanup_sent_dir()

    mqtt_pub = MqttPublisher()
    mqtt_pub.connect()
    mqtt_pub.publish_json(
        MQTT_STATUS_TOPIC,
        {
            "ts": utc_now(),
            "state": "starting",
            "serial_port": SERIAL_PORT,
            "serial_baud": SERIAL_BAUD,
            "protobuf_enabled": mesh_pb2 is not None,
        },
    )

    reader = FrameReader()
    try:
        with serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1) as ser:
            logging.info("[serial] open %s @ %d", SERIAL_PORT, SERIAL_BAUD)
            mqtt_pub.publish_json(
                MQTT_STATUS_TOPIC,
                {
                    "ts": utc_now(),
                    "state": "serial-open",
                    "serial_port": SERIAL_PORT,
                    "serial_baud": SERIAL_BAUD,
                },
            )
            while True:
                if mqtt_pub.connected:
                    mqtt_pub.flush_pending()
                maybe_publish_telemetry(mqtt_pub)
                chunk = ser.read(512)
                if not chunk:
                    continue
                for frame in reader.feed(chunk):
                    handle_frame(mqtt_pub, frame)
    except KeyboardInterrupt:
        logging.info("stopped by user")
        return 0
    except Exception as exc:
        logging.exception("meshproxy failed: %s", exc)
        mqtt_pub.publish_json(MQTT_STATUS_TOPIC, {"ts": utc_now(), "state": "error", "error": str(exc)})
        return 1


if __name__ == "__main__":
    sys.exit(main())
