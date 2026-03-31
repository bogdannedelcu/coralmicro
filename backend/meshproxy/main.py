import logging
import sys
import time

from meshtastic.serial_interface import SerialInterface

from commonproxy import utc_now
from meshproxy import config
from meshproxy.common import spool
from meshproxy.mqtt_client import MqttPublisher
from meshproxy.protocol import MeshSdkReceiver, visionmesh_pb2
from meshproxy.telemetry import maybe_publish_telemetry


def main() -> int:
    spool.ensure_dirs()
    spool.cleanup_sent_dir()

    mqtt_pub = MqttPublisher()
    mqtt_pub.connect()
    mqtt_pub.publish_json(
        config.MQTT_STATUS_TOPIC,
        {
            'ts': utc_now(),
            'state': 'starting',
            'serial_port': config.SERIAL_PORT,
            'serial_baud': 115200,
            'protobuf_enabled': visionmesh_pb2 is not None,
            'transport': 'meshtastic-python-sdk',
        },
    )

    iface = None
    try:
        iface = SerialInterface(devPath=config.SERIAL_PORT, noProto=False, connectNow=True, noNodes=False, timeout=30)
        iface.waitForConfig()
        logging.info('[mesh] connected via meshtastic sdk to %s', config.SERIAL_PORT)
        mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {
            'ts': utc_now(),
            'state': 'serial-open',
            'serial_port': config.SERIAL_PORT,
            'serial_baud': 115200,
            'transport': 'meshtastic-python-sdk',
        })

        receiver = MeshSdkReceiver(mqtt_pub)
        receiver.subscribe()

        while True:
            if mqtt_pub.connected:
                mqtt_pub.flush_pending()
            maybe_publish_telemetry(mqtt_pub)
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info('stopped by user')
        return 0
    except Exception as exc:
        logging.exception('meshproxy failed: %s', exc)
        mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'error', 'error': str(exc)})
        return 1
    finally:
        if iface is not None:
            try:
                iface.close()
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())
