import os
from pathlib import Path

from dotenv import load_dotenv

load_dotenv()

MAVLINK_TCP_HOST = os.getenv("MAVLINK_TCP_HOST", "127.0.0.1")
MAVLINK_TCP_PORT = int(os.getenv("MAVLINK_TCP_PORT", "5760"))
MQTT_URL = os.getenv("MQTT_URL", "mqtt://localhost:1883")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_TOPIC_PREFIX = os.getenv("MQTT_TOPIC_PREFIX", "sentai")
PROXY_ID = os.getenv("PROXY_ID", "link01")


def topic_for(node_id: int, suffix: str) -> str:
    """Build per-node topic: sentai/{proxy_id}/{node_hex}/vision, etc."""
    return f"{MQTT_TOPIC_PREFIX}/{PROXY_ID}/{node_id:08x}/{suffix}"


def proxy_topic(suffix: str) -> str:
    """Build proxy-level topic: sentai/{proxy_id}/status, etc."""
    return f"{MQTT_TOPIC_PREFIX}/{PROXY_ID}/{suffix}"


MQTT_STATUS_TOPIC = proxy_topic("status")
MQTT_AVAILABILITY_TOPIC = proxy_topic("availability")
SPOOL_ROOT = Path(os.getenv("SPOOL_ROOT", str(Path(__file__).resolve().parent.parent / "spool")))
PENDING_DIR = SPOOL_ROOT / "pending"
SENT_DIR = SPOOL_ROOT / "Sent"
FORWARD_RAW_BASE64 = str(os.getenv("FORWARD_RAW_BASE64", "true")).lower() in {"1", "true", "yes", "on"}
SENT_RETENTION_DAYS = 7
LINK_RECENT_WINDOW_SECONDS = 600
TELEMETRY_INTERVAL_SECONDS = 60
STATUSTEXT_CHUNK_SIZE = 50
