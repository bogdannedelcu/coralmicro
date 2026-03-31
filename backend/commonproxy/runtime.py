import json
import logging
import os
import platform
import shutil
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Callable, Tuple

import paho.mqtt.client as mqtt

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_mqtt_url(url: str) -> Tuple[str, int]:
    if "://" in url:
        _, rest = url.split("://", 1)
    else:
        rest = url
    if "/" in rest:
        rest = rest.split("/", 1)[0]
    if ":" in rest:
        host, port = rest.rsplit(":", 1)
        return host, int(port)
    return rest, 1883


def get_ip_addresses() -> list:
    try:
        return [x for x in os.popen("hostname -I").read().strip().split() if x]
    except Exception:
        return []


def get_meminfo() -> dict:
    result = {}
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                result[k.strip()] = v.strip()
    except Exception:
        pass
    return result


def get_uptime_seconds() -> float:
    try:
        return float(Path('/proc/uptime').read_text().split()[0])
    except Exception:
        return 0.0


def get_disk_usage() -> dict:
    st = shutil.disk_usage('/')
    return {'total_bytes': st.total, 'used_bytes': st.used, 'free_bytes': st.free}


def get_system_metrics() -> dict:
    meminfo = get_meminfo()
    total_kb = int(meminfo.get('MemTotal', '0 kB').split()[0]) if meminfo.get('MemTotal') else 0
    avail_kb = int(meminfo.get('MemAvailable', '0 kB').split()[0]) if meminfo.get('MemAvailable') else 0
    used_kb = max(total_kb - avail_kb, 0)
    load1, load5, load15 = os.getloadavg()
    return {
        'hostname': platform.node(),
        'ip_addresses': get_ip_addresses(),
        'os': platform.platform(),
        'python': platform.python_version(),
        'cpu_count': os.cpu_count(),
        'loadavg': {'1m': load1, '5m': load5, '15m': load15},
        'memory': {'total_kb': total_kb, 'available_kb': avail_kb, 'used_kb': used_kb},
        'disk_root': get_disk_usage(),
        'uptime_seconds': get_uptime_seconds(),
    }


class RecentFlag:
    def __init__(self, window_seconds: int):
        self.window_seconds = window_seconds
        self._ts = 0.0
        self._lock = Lock()

    def mark(self) -> None:
        with self._lock:
            self._ts = time.time()

    def is_recent(self) -> bool:
        with self._lock:
            return (time.time() - self._ts) <= self.window_seconds if self._ts else False


class TelemetryTicker:
    def __init__(self, interval_seconds: int):
        self.interval_seconds = interval_seconds
        self.last_ts = 0.0

    def should_fire(self) -> bool:
        now = time.time()
        if now - self.last_ts < self.interval_seconds:
            return False
        self.last_ts = now
        return True


@dataclass
class SpoolConfig:
    pending_dir: Path
    sent_dir: Path
    retention_days: int = 7


class Spool:
    def __init__(self, cfg: SpoolConfig):
        self.cfg = cfg

    def ensure_dirs(self) -> None:
        self.cfg.pending_dir.mkdir(parents=True, exist_ok=True)
        self.cfg.sent_dir.mkdir(parents=True, exist_ok=True)

    def cleanup_sent_dir(self) -> None:
        now = time.time()
        max_age = self.cfg.retention_days * 24 * 3600
        for path in self.cfg.sent_dir.glob('*.json'):
            try:
                if now - path.stat().st_mtime > max_age:
                    path.unlink(missing_ok=True)
            except Exception as exc:
                logging.warning('[spool] cleanup failed for %s: %s', path, exc)

    def write(self, topic: str, payload: dict) -> Path:
        self.ensure_dirs()
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        path = self.cfg.pending_dir / f'{stamp}_{uuid.uuid4().hex}.json'
        env = {'topic': topic, 'payload': payload, 'queued_at': utc_now()}
        path.write_text(json.dumps(env, ensure_ascii=False), encoding='utf-8')
        return path

    def mark_sent(self, path: Path) -> None:
        self.ensure_dirs()
        shutil.move(str(path), str(self.cfg.sent_dir / path.name))

    def flush(self, publish_fn: Callable[[str, dict], bool]) -> None:
        self.ensure_dirs()
        self.cleanup_sent_dir()
        pending = sorted(self.cfg.pending_dir.glob('*.json'))
        if pending:
            logging.info('[spool] flushing %d queued messages', len(pending))
        for path in pending:
            try:
                env = json.loads(path.read_text(encoding='utf-8'))
                ok = publish_fn(env['topic'], env['payload'])
                if ok:
                    self.mark_sent(path)
                else:
                    break
            except Exception as exc:
                logging.warning('[spool] flush failed for %s: %s', path.name, exc)
                break


@dataclass
class MqttConfig:
    url: str
    username: str
    password: str


class MqttPublisher:
    def __init__(self, cfg: MqttConfig, spool: Spool, on_connect: Callable[[], None] | None = None):
        self.cfg = cfg
        self.spool = spool
        self.connected = False
        self._on_connect = on_connect
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if cfg.username:
            self.client.username_pw_set(cfg.username, cfg.password or None)
        self.client.on_connect = self._handle_connect
        self.client.on_disconnect = self._handle_disconnect

    def connect(self) -> None:
        host, port = parse_mqtt_url(self.cfg.url)
        self.client.connect_async(host, port, keepalive=60)
        self.client.loop_start()

    def _handle_connect(self, client, userdata, flags, reason_code, properties):
        self.connected = True
        logging.info('[mqtt] connected to %s', self.cfg.url)
        if self._on_connect:
            self._on_connect()
        self.flush_pending()

    def _handle_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        logging.warning('[mqtt] disconnected: %s', reason_code)

    def _publish_direct(self, topic: str, payload: dict) -> bool:
        info = self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
        info.wait_for_publish(timeout=5)
        return info.rc == mqtt.MQTT_ERR_SUCCESS and info.is_published()

    def publish_json(self, topic: str, payload: dict) -> None:
        if not self.connected:
            path = self.spool.write(topic, payload)
            logging.info('[spool] queued %s -> %s', topic, path.name)
            return
        if not self._publish_direct(topic, payload):
            path = self.spool.write(topic, payload)
            logging.warning('[mqtt] publish failed, queued %s -> %s', topic, path.name)

    def flush_pending(self) -> None:
        self.spool.flush(self._publish_direct)
