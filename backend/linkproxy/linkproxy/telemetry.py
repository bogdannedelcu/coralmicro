from datetime import datetime, timezone

from commonproxy import TelemetryTicker, get_system_metrics

from . import config
from .common import recent_rx

ticker = TelemetryTicker(config.TELEMETRY_INTERVAL_SECONDS)


def build_telemetry() -> dict:
    now = datetime.now(timezone.utc)
    return {
        'ts': now.isoformat(),
        'date': now.strftime('%Y-%m-%d'),
        'time': now.strftime('%H:%M:%S'),
        'kind': 'availability',
        'mavlink_tcp_host': config.MAVLINK_TCP_HOST,
        'mavlink_tcp_port': config.MAVLINK_TCP_PORT,
        'mavlink_message_received_last_10m': recent_rx.is_recent(),
        'system': get_system_metrics(),
    }


def maybe_publish_telemetry(mqtt_pub) -> None:
    if ticker.should_fire():
        mqtt_pub.publish_json(config.MQTT_AVAILABILITY_TOPIC, build_telemetry())
