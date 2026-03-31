from datetime import datetime, timezone

from serial.tools import list_ports

from commonproxy import TelemetryTicker, get_system_metrics

from . import config
from .common import recent_rx

ticker = TelemetryTicker(config.TELEMETRY_INTERVAL_SECONDS)


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
        'mesh_message_received_last_10m': recent_rx.is_recent(),
        'system': get_system_metrics(),
    }


def maybe_publish_telemetry(mqtt_pub) -> None:
    if ticker.should_fire():
        mqtt_pub.publish_json(config.MQTT_AVAILABILITY_TOPIC, build_telemetry())
