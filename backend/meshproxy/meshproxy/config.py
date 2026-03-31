import os
from pathlib import Path

from dotenv import load_dotenv

load_dotenv()

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "38400"))
MQTT_URL = os.getenv("MQTT_URL", "mqtt://localhost:1883")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_VISION_TOPIC = os.getenv("MQTT_VISION_TOPIC", "sentai/vision")
MQTT_TEXT_TOPIC = os.getenv("MQTT_TEXT_TOPIC", "sentai/text")
MQTT_RAW_TOPIC = os.getenv("MQTT_RAW_TOPIC", "sentai/raw")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "sentai/status")
MQTT_AVAILABILITY_TOPIC = os.getenv("MQTT_AVAILABILITY_TOPIC", "sentai/availability")
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
