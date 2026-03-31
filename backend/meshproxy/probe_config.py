import sys, time, struct
from pathlib import Path
sys.path.insert(0, '/home/bogdan/work/coralmicro/backend/meshproxy')
from meshproxy.protocol import FrameReader
from meshtastic import mesh_pb2
import serial

START1=0x94; START2=0xC3

def frame(payload: bytes) -> bytes:
    return bytes([START1, START2]) + struct.pack('>H', len(payload)) + payload

ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.5)
try:
    tr = mesh_pb2.ToRadio()
    tr.want_config_id = 1
    payload = tr.SerializeToString()
    ser.write(frame(payload))
    ser.flush()
    print('sent want_config_id=1')
    reader = FrameReader()
    deadline = time.time() + 8
    seen = 0
    while time.time() < deadline:
        chunk = ser.read(512)
        if not chunk:
            continue
        for fp in reader.feed(chunk):
            try:
                fr = mesh_pb2.FromRadio()
                fr.ParseFromString(fp)
                print(fr)
                seen += 1
                if fr.HasField('config_complete_id'):
                    print('config_complete_id=', fr.config_complete_id)
                    raise SystemExit(0)
            except Exception as e:
                print('decode-failed', e)
    print('done, seen=', seen)
finally:
    ser.close()
