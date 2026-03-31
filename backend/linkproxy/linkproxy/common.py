import json
import logging
import os
import platform
import shutil
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Tuple

from . import config

LAST_RX_TS = 0.0
LAST_RX_LOCK = Lock()
LAST_TELEMETRY_TS = 0.0

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def ensure_dirs() -> None:
    config.PENDING_DIR.mkdir(parents=True, exist_ok=True)
    config.SENT_DIR.mkdir(parents=True, exist_ok=True)


def cleanup_sent_dir() -> None:
    now = time.time()
    max_age = config.SENT_RETENTION_DAYS * 24 * 3600
    for path in config.SENT_DIR.glob("*.json"):
        try:
            if now - path.stat().st_mtime > max_age:
                path.unlink(missing_ok=True)
        except Exception as exc:
            logging.warning("[spool] cleanup failed for %s: %s", path, exc)


def write_spool(topic: str, payload: dict) -> Path:
    ensure_dirs()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    path = config.PENDING_DIR / f"{stamp}_{uuid.uuid4().hex}.json"
    path.write_text(json.dumps({"topic": topic, "payload": payload, "queued_at": utc_now()}, ensure_ascii=False), encoding="utf-8")
    return path


def mark_sent(path: Path) -> None:
    ensure_dirs()
    shutil.move(str(path), str(config.SENT_DIR / path.name))


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


def mark_rx() -> None:
    global LAST_RX_TS
    with LAST_RX_LOCK:
        LAST_RX_TS = time.time()


def has_recent_rx() -> bool:
    with LAST_RX_LOCK:
        return (time.time() - LAST_RX_TS) <= config.LINK_RECENT_WINDOW_SECONDS if LAST_RX_TS else False


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
