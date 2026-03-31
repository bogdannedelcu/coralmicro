import logging
import sys

import serial

from meshproxy import config
from meshproxy.common import cleanup_sent_dir, ensure_dirs, utc_now
from meshproxy.mqtt_client import MqttPublisher
from meshproxy.protocol import FrameReader, handle_frame, mesh_pb2
from meshproxy.telemetry import maybe_publish_telemetry


def main() -> int:
    ensure_dirs()
    cleanup_sent_dir()

    mqtt_pub = MqttPublisher()
    mqtt_pub.connect()
    mqtt_pub.publish_json(
        config.MQTT_STATUS_TOPIC,
        {
            'ts': utc_now(),
            'state': 'starting',
            'serial_port': config.SERIAL_PORT,
            'serial_baud': config.SERIAL_BAUD,
            'protobuf_enabled': mesh_pb2 is not None,
        },
    )

    reader = FrameReader()
    try:
        with serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=1) as ser:
            logging.info('[serial] open %s @ %d', config.SERIAL_PORT, config.SERIAL_BAUD)
            mqtt_pub.publish_json(
                config.MQTT_STATUS_TOPIC,
                {
                    'ts': utc_now(),
                    'state': 'serial-open',
                    'serial_port': config.SERIAL_PORT,
                    'serial_baud': config.SERIAL_BAUD,
                },
            )
            while True:
                if mqtt_pub.connected:
                    mqtt_pub.flush_pending()
                maybe_publish_telemetry(mqtt_pub)
                chunk = ser.read(512)
                if not chunk:
                    continue
                for frame in reader.feed(chunk):
                    handle_frame(mqtt_pub, frame)
    except KeyboardInterrupt:
        logging.info('stopped by user')
        return 0
    except Exception as exc:
        logging.exception('meshproxy failed: %s', exc)
        mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'error', 'error': str(exc)})
        return 1


if __name__ == '__main__':
    sys.exit(main())
