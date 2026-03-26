import glob
import time
from datetime import datetime
import serial

LOG_FILE = "sentai_serial.log"
BAUD = 115200

def ts():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def log_meta(msg: str):
    line = f"[{ts()}] {msg}"
    print(line)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")

def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None

def clean_bytes(data: bytes) -> str:
    # păstrează newline/tab și caractere ASCII imprimabile
    filtered = bytearray()
    for b in data:
        if b in (9, 10, 13) or 32 <= b <= 126:
            filtered.append(b)
    return filtered.decode("ascii", errors="ignore").strip()

while True:
    port = find_port()
    if not port:
        log_meta("No serial device found, retrying...")
        time.sleep(1)
        continue

    try:
        log_meta(f"Connecting to {port}")
        with serial.Serial(port, BAUD, timeout=1) as ser, open(LOG_FILE, "a", encoding="utf-8") as f:
            buffer = b""
            while True:
                chunk = ser.read(256)
                if not chunk:
                    continue

                buffer += chunk

                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    line = clean_bytes(line_bytes)
                    if line:
                        out = f"[{ts()}] {line}"
                        print(out)
                        f.write(out + "\n")
                        f.flush()

    except (serial.SerialException, OSError) as e:
        log_meta(f"Disconnected: {e}")
        time.sleep(1)
    except KeyboardInterrupt:
        log_meta("Stopped by user")
        break