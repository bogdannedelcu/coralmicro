# _t_embedded_id.py — probe OV5640 embedded-data line for per-camera
# fingerprint.  Standard OV5640 features:
#   - reg 0x501F bit 7 = enable embedded data (metadata in line 0)
#   - reg 0x4707 = embedded line count
#   - reg 0x3503 = AEC manual mode (bit 0 = AEC manual, bit 1 = AGC manual)
#   - reg 0x3508/0x3509 = AGC gain real registers (16-bit)
#
# Plan:
#   1. Read current 0x501F on each cam
#   2. Set bit 7 = 1 on both
#   3. Set 0x3503 manual + assign DISTINCT manual gain per cam (cam0=0x10,
#      cam1=0x40 — small visual diff but radically different in raw bytes)
#   4. Capture frame from each cam and dump first 64 bytes
#   5. Look for diverging bytes — that's our hardware fingerprint
import sentai
sentai.verbose(1)

print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)


def hex_dump(b, n=64):
    return " ".join("%02X" % x for x in b[:n])


# Step 1: read current state of relevant registers per cam.
print("--- baseline reg dump ---")
for cid in (0, 1):
    sentai.camera.select(cid)
    sentai.rtos.sleep_ms(150)
    r501f = sentai.camera.reg_read(cid, 0x501F)
    r4707 = sentai.camera.reg_read(cid, 0x4707)
    r3503 = sentai.camera.reg_read(cid, 0x3503)
    r3508 = sentai.camera.reg_read(cid, 0x3508)
    r3509 = sentai.camera.reg_read(cid, 0x3509)
    print("  cam%d 0x501F=0x%02X 0x4707=0x%02X 0x3503=0x%02X 0x3508=0x%02X 0x3509=0x%02X"
          % (cid, r501f, r4707, r3503, r3508, r3509))


# Step 2: enable embedded data on both cameras
print("--- enabling embedded data ---")
for cid in (0, 1):
    cur = sentai.camera.reg_read(cid, 0x501F)
    sentai.camera.reg_write(cid, 0x501F, cur | 0x80)  # bit 7 = embedded data
    sentai.camera.reg_write(cid, 0x4707, 0x01)        # 1 embedded line
    new = sentai.camera.reg_read(cid, 0x501F)
    print("  cam%d 0x501F: 0x%02X -> 0x%02X" % (cid, cur, new))


# Step 3: set DISTINCT manual AGC per camera so the pixel data
# itself differs in a controlled, recognizable way.
# 0x3503 bit 1 = AGC manual; bit 0 = AEC manual.  Set both.
print("--- distinct manual AGC per cam ---")
sentai.camera.reg_write(0, 0x3503, 0x03)  # both manual
sentai.camera.reg_write(0, 0x3508, 0x00)
sentai.camera.reg_write(0, 0x3509, 0x10)  # cam0 gain low (~1x)
sentai.camera.reg_write(1, 0x3503, 0x03)
sentai.camera.reg_write(1, 0x3508, 0x00)
sentai.camera.reg_write(1, 0x3509, 0x80)  # cam1 gain high (~8x)
print("  cam0 gain=0x10  cam1 gain=0x80")


# Step 4: capture from each, dump first row bytes
print("--- peek first 64 bytes per cam ---")
for cid in (0, 1):
    sentai.camera.select(cid)
    sentai.rtos.sleep_ms(200)  # let MUX + AEC settle
    # Drain a couple frames so we don't see the very first stale buffer
    for _ in range(3):
        b = sentai.camera.peek_row(64)
    print("  cam%d row0: %s" % (cid, hex_dump(b)))


# Step 5: revert AGC to auto so we don't leave the camera weird
print("--- restore AEC/AGC auto ---")
for cid in (0, 1):
    sentai.camera.reg_write(cid, 0x3503, 0x00)
    sentai.camera.reg_write(cid, 0x501F,
                            sentai.camera.reg_read(cid, 0x501F) & 0x7F)

print("=== done ===")
