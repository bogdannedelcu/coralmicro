import json
import logging

import paho.mqtt.client as mqtt

from . import config
from .common import cleanup_sent_dir, ensure_dirs, mark_sent, parse_mqtt_url, write_spool, utc_now


class MqttPublisher:
    def __init__(self) -> None:
        self.connected = False
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if config.MQTT_USERNAME:
            self.client.username_pw_set(config.MQTT_USERNAME, config.MQTT_PASSWORD or None)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

    def connect(self) -> None:
        host, port = parse_mqtt_url(config.MQTT_URL)
        self.client.connect_async(host, port, keepalive=60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.connected = True
        logging.info('[mqtt] connected to %s', config.MQTT_URL)
        self.publish_json(config.MQTT_STATUS_TOPIC, {
            'ts': utc_now(),
            'state': 'connected',
            'serial_port': config.SERIAL_PORT,
            'serial_baud': config.SERIAL_BAUD,
        })
        self.flush_pending()

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        logging.warning('[mqtt] disconnected: %s', reason_code)

    def publish_json(self, topic: str, payload: dict) -> None:
        if not self.connected:
            path = write_spool(topic, payload)
            logging.info('[spool] queued %s -> %s', topic, path.name)
            return
        info = self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
        info.wait_for_publish(timeout=5)
        if info.rc != mqtt.MQTT_ERR_SUCCESS or not info.is_published():
            path = write_spool(topic, payload)
            logging.warning('[mqtt] publish failed, queued %s -> %s', topic, path.name)

    def flush_pending(self) -> None:
        ensure_dirs()
        cleanup_sent_dir()
        pending = sorted(config.PENDING_DIR.glob('*.json'))
        if pending:
            logging.info('[spool] flushing %d queued messages', len(pending))
        for path in pending:
            try:
                env = json.loads(path.read_text(encoding='utf-8'))
                info = self.client.publish(env['topic'], json.dumps(env['payload'], ensure_ascii=False), qos=1, retain=False)
                info.wait_for_publish(timeout=5)
                if info.rc == mqtt.MQTT_ERR_SUCCESS and info.is_published():
                    mark_sent(path)
                else:
                    break
            except Exception as exc:
                logging.warning('[spool] flush failed for %s: %s', path.name, exc)
                break
