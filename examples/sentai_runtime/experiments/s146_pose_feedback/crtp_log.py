# crtp_log.py — pure-MP CRTP LOG subscription for cf2 SITL.
#
# Task #41 (Option B): no new C code; runs entirely in MP using the
# sentai.crazy.send_crtp + recv_crtp primitives shipped in Task #39.
#
# Protocol (mirrors cflib/crazyflie/{log,toc}.py wire format):
#   port=0x05 (LOGGING) channel=0 (TOC)       — TOC info + items (V2)
#   port=0x05            channel=1 (SETTINGS) — block create / start / stop
#   port=0x05            channel=2 (LOGDATA)  — drone-→host data frames
#
# Wire format on the UDP socket (each datagram is one CRTP packet):
#   [header_byte] [payload_bytes_0..30]
#   header = (port<<4) | 0x0C | (channel & 0x03)
#
# Typical usage:
#   import crtp_log
#   crtp_log.scan_toc()                                  # ~1-3 s
#   bid = crtp_log.create_pose_block(period_ms=100)      # subscribe
#   ...
#   pose = crtp_log.latest_pose()                        # (x,y,z,yaw) | None
#   ...
#   crtp_log.stop(bid)
#
# All scan / create-block calls are blocking with a per-request timeout
# (default 500 ms).  poll-style latest_*() calls are non-blocking.
#
# What we deliberately don't implement (yet):
#   - TOC persistence to disk (each process re-scans ~1-3 s)
#   - Memory-mapped log vars (we only do TOC variables)
#   - LOG block append (we cap at one create per block)

import sentai
import struct

# ─── Protocol constants (verbatim from cflib/crazyflie/log.py) ─────
CRTP_PORT_LOG       = 0x05
CH_TOC              = 0
CH_SETTINGS         = 1
CH_LOGDATA          = 2

CMD_GET_ITEM_V2     = 2
CMD_GET_INFO_V2     = 3

CMD_CREATE_BLOCK_V2 = 6
CMD_APPEND_BLOCK_V2 = 7
CMD_DELETE_BLOCK    = 2
CMD_START_LOGGING   = 3
CMD_STOP_LOGGING    = 4
CMD_RESET_LOGGING   = 5

# LogTocElement types — only the ones we care about.
LOG_T_UINT8   = 0x01
LOG_T_UINT16  = 0x02
LOG_T_UINT32  = 0x03
LOG_T_INT8    = 0x04
LOG_T_INT16   = 0x05
LOG_T_INT32   = 0x06
LOG_T_FLOAT   = 0x07
LOG_T_FP16    = 0x08

# Type → (struct format, byte size).  Used for parsing LOGDATA payload.
_TYPE_INFO = {
    LOG_T_UINT8:  ('<B', 1),
    LOG_T_UINT16: ('<H', 2),
    LOG_T_UINT32: ('<L', 4),
    LOG_T_INT8:   ('<b', 1),
    LOG_T_INT16:  ('<h', 2),
    LOG_T_INT32:  ('<i', 4),
    LOG_T_FLOAT:  ('<f', 4),
}

# ─── Module state ───────────────────────────────────────────────────
_toc = {}              # (group, name) -> (ident, type_byte)
_blocks = {}           # block_id -> [(group, name, type_byte, fmt, sz), ...]
_latest_pose = None    # (x, y, z, yaw) — populated by poll()


# ─── Wire-format builders (pure functions; testable offline) ───────

def _build_get_info_pkt():
    """One byte: CMD_GET_INFO_V2."""
    return bytes([CMD_GET_INFO_V2])


def _build_get_item_pkt(idx):
    """3 bytes: CMD_GET_ITEM_V2 + idx_lo + idx_hi."""
    return bytes([CMD_GET_ITEM_V2, idx & 0xFF, (idx >> 8) & 0xFF])


def _build_create_block_pkt(block_id, var_entries):
    """Build a CMD_CREATE_BLOCK_V2 packet.
    var_entries = [(ident:int, fetch_type:int), ...]
    Wire (V2): [cmd, block_id, fetch_byte, id_lo, id_hi, ...]
    fetch_byte = (fetch_as | stored_as<<4) — same type for both ⇒ fetch | fetch<<4.
    Built via plain `list` because MP embed has no `bytearray`.
    """
    parts = [CMD_CREATE_BLOCK_V2, block_id & 0xFF]
    for (ident, t) in var_entries:
        fetch_byte = (t & 0x0F) | ((t & 0x0F) << 4)
        parts.append(fetch_byte)
        parts.append(ident & 0xFF)
        parts.append((ident >> 8) & 0xFF)
    return bytes(parts)


def _build_start_pkt(block_id, period_10ms):
    """3 bytes: CMD_START_LOGGING + block_id + period (in 10ms units)."""
    return bytes([CMD_START_LOGGING, block_id & 0xFF, period_10ms & 0xFF])


def _build_stop_pkt(block_id):
    return bytes([CMD_STOP_LOGGING, block_id & 0xFF])


def _build_delete_pkt(block_id):
    return bytes([CMD_DELETE_BLOCK, block_id & 0xFF])


def _build_reset_pkt():
    return bytes([CMD_RESET_LOGGING])


# ─── RX dispatch (drains the sentai.crazy FIFO) ────────────────────

def _drain_for_reply(channel, cmd, timeout_ms=500):
    """Send/wait helper.  Drains recv_crtp until a packet matching
    (CRTP_PORT_LOG, channel, payload[0]==cmd) appears, OR timeout.
    Returns the matching payload (with cmd byte stripped) or None."""
    polled_ms = 0
    while polled_ms < timeout_ms:
        pkt = sentai.crazy.recv_crtp()
        if pkt is None:
            sentai.rtos.sleep_ms(5)
            polled_ms += 5
            continue
        port, ch, data = pkt
        if port == CRTP_PORT_LOG and ch == CH_LOGDATA:
            # Log data frame — buffer separately for poll() consumers.
            _consume_logdata(data)
            continue
        if port == CRTP_PORT_LOG and ch == channel and len(data) >= 1 and data[0] == cmd:
            return data[1:]
        # Some other packet (different port/channel) — drop silently.
    return None


def _consume_logdata(data):
    """Parse a LOGDATA packet and store the values.  Format:
       [block_id] [ts_lo] [ts_mid] [ts_hi] [var_data...]"""
    global _latest_pose
    if len(data) < 4:
        return
    block_id = data[0]
    # ts = data[1] | data[2]<<8 | data[3]<<16  -- we ignore for now
    spec = _blocks.get(block_id)
    if spec is None:
        return
    off = 4
    values = []
    for (group, name, t, fmt, sz) in spec:
        if off + sz > len(data):
            return
        v = struct.unpack(fmt, data[off:off+sz])[0]
        values.append(v)
        off += sz
    # Build a (x,y,z,yaw) tuple if this is the POSE block.
    if spec and len(values) == 4 and \
       spec[0][1] == 'x' and spec[1][1] == 'y' and \
       spec[2][1] == 'z' and spec[3][1] == 'yaw':
        _latest_pose = (values[0], values[1], values[2], values[3])


# ─── High-level API ─────────────────────────────────────────────────

def reset():
    """Send RESET_LOGGING — clears all blocks on the cf2.  Idempotent."""
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_SETTINGS, _build_reset_pkt())
    _drain_for_reply(CH_SETTINGS, CMD_RESET_LOGGING, timeout_ms=200)


def scan_toc(timeout_ms=3000, max_items=400):
    """Discover (group, name) → ID for every log variable.
    Returns the number of entries discovered.  Idempotent (re-scans)."""
    _toc.clear()

    # 1. INFO request → 6-byte payload (num_items_lo, num_items_hi, crc[4])
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_TOC, _build_get_info_pkt())
    info = _drain_for_reply(CH_TOC, CMD_GET_INFO_V2, timeout_ms=1000)
    if info is None:
        return -1
    if len(info) < 6:
        return -2
    n_items = info[0] | (info[1] << 8)
    if n_items > max_items:
        n_items = max_items

    # 2. For each index 0..n_items-1, send GET_ITEM_V2 and parse reply.
    #    Reply payload (after cmd byte): [idx_lo, idx_hi, type, group\0, name\0]
    for i in range(n_items):
        sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_TOC, _build_get_item_pkt(i))
        item = _drain_for_reply(CH_TOC, CMD_GET_ITEM_V2, timeout_ms=200)
        if item is None or len(item) < 5:
            continue
        idx = item[0] | (item[1] << 8)
        if idx != i:
            continue
        ttype = item[2]
        rest = item[3:]
        # Split on NUL: [group_bytes, NUL, name_bytes, NUL]
        nul1 = rest.find(b'\x00')
        if nul1 < 0:
            continue
        # MP embed `bytes` lacks .decode(); str(b, 'ascii') is the
        # portable equivalent.  Group/name are guaranteed ASCII by
        # the cf2 firmware-side LOG_ADD macros (C identifier subset).
        group = str(rest[:nul1], 'ascii')
        nul2 = rest.find(b'\x00', nul1 + 1)
        if nul2 < 0:
            continue
        name = str(rest[nul1+1:nul2], 'ascii')
        _toc[(group, name)] = (idx, ttype)
    return len(_toc)


def find(group, name):
    """Lookup TOC entry by (group, name).  Returns (id, type) or None."""
    return _toc.get((group, name))


def create_pose_block(block_id=1, period_ms=100):
    """Subscribe to stateEstimate.{x,y,z} + stabilizer.yaw.
    Returns block_id on success, negative error code otherwise.

    Preconditions: scan_toc() must have completed.
    Side effect: a subsequent latest_pose() drains LOGDATA into a
    cached (x,y,z,yaw) tuple."""
    targets = [
        ('stateEstimate', 'x'),
        ('stateEstimate', 'y'),
        ('stateEstimate', 'z'),
        ('stabilizer',    'yaw'),
    ]
    var_entries = []
    spec = []
    for (g, n) in targets:
        entry = _toc.get((g, n))
        if entry is None:
            return -10                # var not in TOC
        ident, ttype = entry
        if ttype not in _TYPE_INFO:
            return -11                # unsupported type
        var_entries.append((ident, ttype))
        fmt, sz = _TYPE_INFO[ttype]
        spec.append((g, n, ttype, fmt, sz))

    # CREATE_BLOCK_V2
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_SETTINGS,
                            _build_create_block_pkt(block_id, var_entries))
    rep = _drain_for_reply(CH_SETTINGS, CMD_CREATE_BLOCK_V2, timeout_ms=500)
    if rep is None:
        return -20
    # rep = [block_id, err_code]  (err_code 0 = ok)
    if len(rep) < 2 or rep[0] != block_id or rep[1] != 0:
        # Block may already exist — re-use silently.
        # (cflib treats EEXIST=17 as recoverable; we do the same.)
        if not (len(rep) >= 2 and rep[1] == 17):
            return -21

    # START_LOGGING
    period_10ms = max(1, period_ms // 10)
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_SETTINGS,
                            _build_start_pkt(block_id, period_10ms))
    rep = _drain_for_reply(CH_SETTINGS, CMD_START_LOGGING, timeout_ms=500)
    if rep is None:
        return -30
    if len(rep) < 2 or rep[0] != block_id or rep[1] != 0:
        return -31

    _blocks[block_id] = spec
    return block_id


def poll(max_pkts=8):
    """Drain up to `max_pkts` LOGDATA packets from the recv FIFO.
    Each one is decoded and updates `_latest_pose` (or any other future
    cached state).  Returns number drained.  Non-blocking."""
    n = 0
    while n < max_pkts:
        pkt = sentai.crazy.recv_crtp()
        if pkt is None:
            return n
        port, ch, data = pkt
        if port == CRTP_PORT_LOG and ch == CH_LOGDATA:
            _consume_logdata(data)
        n += 1
    return n


def latest_pose():
    """Return the most recently received (x, y, z, yaw) tuple, or None
    if no LOGDATA frame has been processed since module load.

    Caller MUST call poll() periodically to drive the FIFO drain.
    (Or build the equivalent into their mission loop.)"""
    poll()
    return _latest_pose


def stop(block_id):
    """Stop + delete a log block.  Idempotent."""
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_SETTINGS,
                            _build_stop_pkt(block_id))
    _drain_for_reply(CH_SETTINGS, CMD_STOP_LOGGING, timeout_ms=200)
    sentai.crazy.send_crtp(CRTP_PORT_LOG, CH_SETTINGS,
                            _build_delete_pkt(block_id))
    _drain_for_reply(CH_SETTINGS, CMD_DELETE_BLOCK, timeout_ms=200)
    _blocks.pop(block_id, None)
