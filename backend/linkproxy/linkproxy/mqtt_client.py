from commonproxy import MqttConfig, MqttPublisher as BaseMqttPublisher, utc_now

from . import config
from .common import spool


class MqttPublisher(BaseMqttPublisher):
    def __init__(self) -> None:
        super().__init__(
            MqttConfig(config.MQTT_URL, config.MQTT_USERNAME, config.MQTT_PASSWORD),
            spool,
            on_connect=self._publish_connected_status,
        )

    def _publish_connected_status(self) -> None:
        self.publish_json(config.MQTT_STATUS_TOPIC, {
            'ts': utc_now(),
            'state': 'connected',
            'mavlink_tcp_host': config.MAVLINK_TCP_HOST,
            'mavlink_tcp_port': config.MAVLINK_TCP_PORT,
        })
