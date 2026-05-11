"""Sniff /tmp/sentai_flow_out.sock for 12 s, classify conf distribution.

Stand-alone tool: connect to bridge output, read every flow_reply for
a window, bucket by conf and by |dx|+|dy| magnitude.  Reveals whether
slow-drift frames produce conf=0 (hypothesis being tested).
"""
import socket, struct, time, sys
from collections import Counter

REPLY_FMT = "<IIiiIQiI"
REPLY_SZ = struct.calcsize(REPLY_FMT)
REPLY_MAGIC = 0x46524C31

DURATION_S = 12.0
SOCK_PATH = "/tmp/sentai_flow_out.sock"

s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(SOCK_PATH)
s.settimeout(0.5)
print(f"[sniff] connected to {SOCK_PATH}, sniffing {DURATION_S}s", file=sys.stderr)

t0 = time.monotonic()
buf = b""
records = []
while time.monotonic() - t0 < DURATION_S:
    try:
        chunk = s.recv(4096)
        if not chunk: continue
        buf += chunk
        while len(buf) >= REPLY_SZ:
            rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
            magic, seq, dx, dy, conf, lat, dz, dz_conf = struct.unpack(REPLY_FMT, rec)
            if magic != REPLY_MAGIC:
                idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                buf = buf[idx:] if idx >= 0 else b""
                continue
            records.append((dx, dy, conf))
    except socket.timeout:
        continue

# Analyse
print(f"\n=== flow_reply distribution over {DURATION_S}s ({len(records)} records) ===")
conf_zero = sum(1 for r in records if r[2] == 0)
conf_low = sum(1 for r in records if 0 < r[2] < 64)
conf_med = sum(1 for r in records if 64 <= r[2] < 128)
conf_high = sum(1 for r in records if r[2] >= 128)
print(f"conf=0         : {conf_zero:>4} ({conf_zero/len(records)*100:5.1f}%)  [EKF down-weights]")
print(f"conf 1-63      : {conf_low:>4} ({conf_low/len(records)*100:5.1f}%)  [low trust]")
print(f"conf 64-127    : {conf_med:>4} ({conf_med/len(records)*100:5.1f}%)  [medium]")
print(f"conf 128-255   : {conf_high:>4} ({conf_high/len(records)*100:5.1f}%)  [high trust]")

# Motion-magnitude buckets
def mag_bucket(dx, dy):
    m = max(abs(dx), abs(dy))
    if m == 0: return "zero"
    if m < 100: return "<100 mgrid (sub-pixel)"
    if m < 500: return "100-500 mgrid"
    if m < 1500: return "500-1500 mgrid"
    return ">=1500 mgrid"

mag_counter = Counter(mag_bucket(r[0], r[1]) for r in records)
print(f"\n--- by motion magnitude (max|dx|,|dy|) ---")
for bucket in ["zero", "<100 mgrid (sub-pixel)", "100-500 mgrid", "500-1500 mgrid", ">=1500 mgrid"]:
    n = mag_counter.get(bucket, 0)
    print(f"{bucket:>30s}: {n:>4} ({n/len(records)*100:5.1f}%)")

# Conf vs mag cross-tab
print(f"\n--- cross: % of frames with conf=0 by motion bucket ---")
for bucket in ["zero", "<100 mgrid (sub-pixel)", "100-500 mgrid", "500-1500 mgrid"]:
    matches = [r for r in records if mag_bucket(r[0], r[1]) == bucket]
    if not matches: continue
    n_zero = sum(1 for r in matches if r[2] == 0)
    print(f"{bucket:>30s}: {n_zero}/{len(matches)} = {n_zero/len(matches)*100:.1f}% have conf=0")
