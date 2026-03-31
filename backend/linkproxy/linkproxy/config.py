import os
from pathlib import Path

from dotenv import load_dotenv

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
SPOOL_ROOT = Path(os.getenv("SPOOL_ROOT", str(Path(__file__).resolve().parent.parent / "spool")))
PENDING_DIR = SPOOL_ROOT / "pending"
SENT_DIR = SPOOL_ROOT / "Sent"
FORWARD_RAW_BASE64 = str(os.getenv("FORWARD_RAW_BASE64", "true")).lower() in {"1", "true", "yes", "on"}
SENT_RETENTION_DAYS = 7
LINK_RECENT_WINDOW_SECONDS = 600
TELEMETRY_INTERVAL_SECONDS = 60
STATUSTEXT_CHUNK_SIZE = 50
