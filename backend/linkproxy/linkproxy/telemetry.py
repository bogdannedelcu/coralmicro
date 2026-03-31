import time
from datetime import datetime, timezone

from . import config
from .common import get_system_metrics, has_recent_rx


def build_telemetry() -> dict:
    now = datetime.now(timezone.utc)
    return {
        'ts': now.isoformat(),
        'date': now.strftime('%Y-%m-%d'),
        'time': now.strftime('%H:%M:%S'),
        'kind': 'availability',
        'mavlink_tcp_host': config.MAVLINK_TCP_HOST,
        'mavlink_tcp_port': config.MAVLINK_TCP_PORT,
        'mavlink_message_received_last_10m': has_recent_rx(),
        'system': get_system_metrics(),
    }


def maybe_publish_telemetry(mqtt_pub) -> None:
    import linkproxy.common as common
    now = time.time()
    if now - common.LAST_TELEMETRY_TS < config.TELEMETRY_INTERVAL_SECONDS:
        return
    common.LAST_TELEMETRY_TS = now
    mqtt_pub.publish_json(config.MQTT_AVAILABILITY_TOPIC, build_telemetry())
