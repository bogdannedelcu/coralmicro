import time
from datetime import datetime, timezone

from serial.tools import list_ports

from . import config
from .common import LAST_TELEMETRY_TS, get_system_metrics, has_recent_rx


def list_serial_ports() -> list:
    return [{"device": p.device, "description": p.description, "hwid": p.hwid} for p in list_ports.comports()]


def build_telemetry() -> dict:
    now = datetime.now(timezone.utc)
    return {
        'ts': now.isoformat(),
        'date': now.strftime('%Y-%m-%d'),
        'time': now.strftime('%H:%M:%S'),
        'kind': 'availability',
        'serial_configured_port': config.SERIAL_PORT,
        'serial_baud': config.SERIAL_BAUD,
        'serial_ports': list_serial_ports(),
        'mesh_message_received_last_10m': has_recent_rx(),
        'system': get_system_metrics(),
    }


def maybe_publish_telemetry(mqtt_pub) -> None:
    import meshproxy.common as common
    now = time.time()
    if now - common.LAST_TELEMETRY_TS < config.TELEMETRY_INTERVAL_SECONDS:
        return
    common.LAST_TELEMETRY_TS = now
    mqtt_pub.publish_json(config.MQTT_AVAILABILITY_TOPIC, build_telemetry())
