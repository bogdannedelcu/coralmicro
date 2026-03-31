import logging
import socket
import sys
import time

from pymavlink import mavutil

from linkproxy import config
from linkproxy.common import cleanup_sent_dir, ensure_dirs, utc_now
from linkproxy.mqtt_client import MqttPublisher
from linkproxy.protocol import StatusTextAssembler, handle_message, mesh_pb2, run_selftest
from linkproxy.telemetry import maybe_publish_telemetry


def main() -> int:
    if '--selftest' in sys.argv:
        return run_selftest()

    ensure_dirs()
    cleanup_sent_dir()

    mqtt_pub = MqttPublisher()
    mqtt_pub.connect()
    mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {
        'ts': utc_now(),
        'state': 'starting',
        'mavlink_tcp_host': config.MAVLINK_TCP_HOST,
        'mavlink_tcp_port': config.MAVLINK_TCP_PORT,
        'protobuf_enabled': mesh_pb2 is not None,
    })

    assembler = StatusTextAssembler()
    endpoint = f'tcp:{config.MAVLINK_TCP_HOST}:{config.MAVLINK_TCP_PORT}'

    while True:
        try:
            conn = mavutil.mavlink_connection(endpoint, source_system=250, source_component=1, autoreconnect=True)
            mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'mavlink-connected', 'endpoint': endpoint})
            logging.info('[link] connected to %s', endpoint)

            while True:
                if mqtt_pub.connected:
                    mqtt_pub.flush_pending()
                maybe_publish_telemetry(mqtt_pub)
                assembler.cleanup()

                msg = conn.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue
                handle_message(mqtt_pub, assembler, msg)
        except KeyboardInterrupt:
            logging.info('stopped by user')
            return 0
        except (socket.error, OSError) as exc:
            logging.warning('[link] connection error: %s', exc)
            mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'mavlink-disconnected', 'error': str(exc), 'endpoint': endpoint})
            time.sleep(2)
        except Exception as exc:
            logging.exception('linkproxy failed: %s', exc)
            mqtt_pub.publish_json(config.MQTT_STATUS_TOPIC, {'ts': utc_now(), 'state': 'error', 'error': str(exc)})
            time.sleep(2)


if __name__ == '__main__':
    sys.exit(main())
