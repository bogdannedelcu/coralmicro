# t_crtp_log_offline.py — wire-format unit tests for crtp_log.py.
#
# Validates byte construction WITHOUT requiring cf2 SITL.  We exercise
# the pure `_build_*` helpers and compare against the expected hex
# strings derived from cflib's wire-format (see crtp_log.py docstring).
#
# This is NOT an end-to-end test.  When Gazebo + cf2 SITL is up, the
# end-to-end test lives at examples/.../s146_s136_migration/

import crtp_log

print("---- t_crtp_log_offline ----")

# 1. CMD_GET_INFO_V2 — single byte 0x03
b = crtp_log._build_get_info_pkt()
assert b == b"\x03", "GET_INFO wrong: %r" % b
print("get_info:", "".join("%02x" % c for c in b))

# 2. CMD_GET_ITEM_V2 idx=42 — [0x02, 0x2A, 0x00]
b = crtp_log._build_get_item_pkt(42)
assert b == b"\x02\x2a\x00", "GET_ITEM wrong: %r" % b
print("get_item(42):", "".join("%02x" % c for c in b))

# 3. CMD_GET_ITEM_V2 idx=300 (multi-byte) — [0x02, 0x2C, 0x01]
b = crtp_log._build_get_item_pkt(300)
assert b == b"\x02\x2c\x01", "GET_ITEM big idx wrong: %r" % b
print("get_item(300):", "".join("%02x" % c for c in b))

# 4. CREATE_BLOCK_V2 block_id=1, var_entry=(ident=0x70, type=FLOAT=7).
#    fetch_byte = (7|7<<4) = 0x77.  Wire: [0x06, 0x01, 0x77, 0x70, 0x00]
b = crtp_log._build_create_block_pkt(1, [(0x70, 7)])
assert b == b"\x06\x01\x77\x70\x00", "CREATE_BLOCK wrong: %r" % b
print("create_block(1, [(0x70, float)]):", "".join("%02x" % c for c in b))

# 5. CREATE_BLOCK_V2 with 4 float vars (canonical pose subscription).
b = crtp_log._build_create_block_pkt(
    2, [(0x70, 7), (0x71, 7), (0x72, 7), (0x49, 7)])
# expected: [cmd=06, blk=02, 0x77,0x70,0x00, 0x77,0x71,0x00, 0x77,0x72,0x00, 0x77,0x49,0x00]
expected = b"\x06\x02" + b"\x77\x70\x00" + b"\x77\x71\x00" + b"\x77\x72\x00" + b"\x77\x49\x00"
assert b == expected, "CREATE_BLOCK 4-var wrong:\n  got %r\n  exp %r" % (b, expected)
print("create_block(2, 4 floats):", "".join("%02x" % c for c in b))

# 6. START_LOGGING block=1 period=10 (= 100ms) — [0x03, 0x01, 0x0A]
b = crtp_log._build_start_pkt(1, 10)
assert b == b"\x03\x01\x0a", "START wrong: %r" % b
print("start(1, 10):", "".join("%02x" % c for c in b))

# 7. STOP_LOGGING block=1 — [0x04, 0x01]
b = crtp_log._build_stop_pkt(1)
assert b == b"\x04\x01", "STOP wrong: %r" % b
print("stop(1):", "".join("%02x" % c for c in b))

# 8. DELETE_BLOCK block=1 — [0x02, 0x01]
b = crtp_log._build_delete_pkt(1)
assert b == b"\x02\x01", "DELETE wrong: %r" % b
print("delete(1):", "".join("%02x" % c for c in b))

# 9. RESET_LOGGING — [0x05]
b = crtp_log._build_reset_pkt()
assert b == b"\x05", "RESET wrong: %r" % b
print("reset:", "".join("%02x" % c for c in b))

# 10. Constant smoke — make sure none drifted.
assert crtp_log.CRTP_PORT_LOG == 0x05
assert crtp_log.CH_TOC == 0
assert crtp_log.CH_SETTINGS == 1
assert crtp_log.CH_LOGDATA == 2
assert crtp_log.CMD_GET_INFO_V2 == 3
assert crtp_log.CMD_GET_ITEM_V2 == 2
assert crtp_log.CMD_CREATE_BLOCK_V2 == 6
assert crtp_log.CMD_START_LOGGING == 3
assert crtp_log.LOG_T_FLOAT == 0x07
print("constants OK")

# 11. Type-info table covers all common cf2 log types.
for tid in (1, 2, 3, 4, 5, 6, 7):
    assert tid in crtp_log._TYPE_INFO, "missing type id %d" % tid
print("type table OK")

print("---- t_crtp_log_offline PASS ----")
