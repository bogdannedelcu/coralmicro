import os
from pathlib import Path

from dotenv import load_dotenv

load_dotenv()

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "38400"))
MQTT_URL = os.getenv("MQTT_URL", "mqtt://localhost:1883")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_TOPIC_PREFIX = os.getenv("MQTT_TOPIC_PREFIX", "sentai")
PROXY_ID = os.getenv("PROXY_ID", "proxy01")


def topic_for(node_id: int, suffix: str) -> str:
    """Build per-node topic: sentai/{proxy_id}/{node_hex}/vision, etc."""
    return f"{MQTT_TOPIC_PREFIX}/{PROXY_ID}/{node_id:08x}/{suffix}"


def proxy_topic(suffix: str) -> str:
    """Build proxy-level topic: sentai/{proxy_id}/status, sentai/{proxy_id}/availability."""
    return f"{MQTT_TOPIC_PREFIX}/{PROXY_ID}/{suffix}"


MQTT_STATUS_TOPIC = proxy_topic("status")
MQTT_AVAILABILITY_TOPIC = proxy_topic("availability")
FORWARD_RAW_BASE64 = str(os.getenv("FORWARD_RAW_BASE64", "true")).lower() in {"1", "true", "yes", "on"}
SPOOL_ROOT = Path(os.getenv("SPOOL_ROOT", str(Path(__file__).resolve().parent.parent / "spool")))
PENDING_DIR = SPOOL_ROOT / "pending"
SENT_DIR = SPOOL_ROOT / "Sent"
SENT_RETENTION_DAYS = 7
MESH_RECENT_WINDOW_SECONDS = 600
TELEMETRY_INTERVAL_SECONDS = 60
START1 = 0x94
START2 = 0xC3
TEXT_MESSAGE_APP = 1
TEXT_MESSAGE_COMPRESSED_APP = 7
DETECTION_SENSOR_APP = 10
ALERT_APP = 11
PRIVATE_APP = 256
