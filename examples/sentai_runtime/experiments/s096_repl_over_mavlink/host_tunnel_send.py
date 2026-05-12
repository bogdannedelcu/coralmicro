"""s096 — Host-side sender: drive sentai_sim REPL over MAVLink TUNNEL.

Sends a command line as TUNNEL message (payload_type=0xC0DE,
"SentAI REPL") to UDP 14540 where sentai_sim's link backend is bound.
sentai_sim's reader task parses the TUNNEL, pushes payload bytes into
the stdin FIFO, MicroPython sees them as if typed locally and runs the
command.

Usage:
    python3 host_tunnel_send.py "print(1+1)"
    python3 host_tunnel_send.py "import sentai; sentai.version()"

Result/output is visible on the sentai_sim console (stdout capture +
back-channel TUNNEL is Phase 6c work, not Phase 6a MVP).
"""
import argparse
import socket
import sys
import time

# MAVLink v2 protocol details (hand-rolled — no pymavlink dep needed for
# a simple TUNNEL injector).
#
# Wire format MAVLink v2:
#  byte 0    = 0xFD start
#  byte 1    = payload length (uint8)
#  byte 2    = incompat_flags (0 = unsigned)
#  byte 3    = compat_flags (0)
#  byte 4    = packet seq
#  byte 5    = sysid
#  byte 6    = compid
#  bytes 7-9 = msgid (uint24, LSB first)
#  bytes 10..10+len-1 = payload
#  bytes len+10, len+11 = CRC16 (X.25 over [byte 1 .. payload end] + msg CRC_EXTRA)
#
# TUNNEL message (msgid 385):
#  uint16 payload_type
#  uint8  target_system
#  uint8  target_component
#  uint8  payload_length
#  uint8  payload[128]
#  Total payload = 133 bytes (= MAVLINK_MSG_ID_TUNNEL_LEN).
# CRC_EXTRA for TUNNEL = 147 (from mavlink_msg_tunnel.h).

MAV_STX_V2     = 0xFD
MSG_ID_TUNNEL  = 385
CRC_EXTRA_TUNNEL = 147
SENTAI_REPL_PAYLOAD_TYPE = 0xC0DE


def x25_crc(data: bytes, init: int = 0xFFFF) -> int:
    """X.25 CRC-16 used by MAVLink."""
    crc = init
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def encode_tunnel(payload_bytes: bytes,
                   payload_type: int = SENTAI_REPL_PAYLOAD_TYPE,
                   target_sys: int = 1,
                   target_comp: int = 191,
                   src_sys: int = 255,
                   src_comp: int = 190,
                   seq: int = 0) -> bytes:
    if len(payload_bytes) > 128:
        raise ValueError(f"payload too large: {len(payload_bytes)} > 128")
    # Build TUNNEL payload (133 bytes total)
    tunnel_pl = bytearray(133)
    # uint16 payload_type @ offset 0
    tunnel_pl[0:2] = payload_type.to_bytes(2, "little")
    # uint8 target_system @ offset 2
    tunnel_pl[2] = target_sys
    # uint8 target_component @ offset 3
    tunnel_pl[3] = target_comp
    # uint8 payload_length @ offset 4
    tunnel_pl[4] = len(payload_bytes)
    # uint8[128] payload starting @ offset 5
    tunnel_pl[5:5 + len(payload_bytes)] = payload_bytes

    # Build MAVLink v2 frame
    header = bytearray(10)
    header[0] = MAV_STX_V2
    header[1] = 133                  # payload length
    header[2] = 0                    # incompat flags
    header[3] = 0                    # compat flags
    header[4] = seq & 0xFF
    header[5] = src_sys
    header[6] = src_comp
    header[7] = MSG_ID_TUNNEL & 0xFF
    header[8] = (MSG_ID_TUNNEL >> 8) & 0xFF
    header[9] = (MSG_ID_TUNNEL >> 16) & 0xFF

    # CRC over bytes [1..end_of_payload] + CRC_EXTRA
    crc_in = bytes(header[1:]) + bytes(tunnel_pl) + bytes([CRC_EXTRA_TUNNEL])
    crc = x25_crc(crc_in)
    crc_bytes = crc.to_bytes(2, "little")

    return bytes(header) + bytes(tunnel_pl) + crc_bytes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", help="REPL command to inject (auto '\\n' appended)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14540,
                    help="sentai_sim UDP bind port (default 14540)")
    ap.add_argument("--sys", type=int, default=1, help="target sysid (sentai default 1)")
    ap.add_argument("--comp", type=int, default=191, help="target compid")
    ap.add_argument("--src-sys", type=int, default=255)
    ap.add_argument("--src-comp", type=int, default=190)
    ap.add_argument("--no-newline", action="store_true",
                    help="don't append \\n to the command")
    args = ap.parse_args()

    cmd = args.command
    # Crazyflie-radio convention: prefix with '$' to mark "this is a
    # command to exec".  sentai_link strips the prefix before pushing
    # bytes into the REPL FIFO.  Lines without '$' are dropped — guards
    # against random TUNNEL bytes triggering arbitrary code.
    if not cmd.startswith("$"):
        cmd = "$" + cmd
    if not args.no_newline and not cmd.endswith("\n"):
        cmd = cmd + "\n"
    payload = cmd.encode("utf-8")

    # Split into chunks of 128 B if needed (TUNNEL MTU).
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    seq = 0
    for i in range(0, len(payload), 128):
        chunk = payload[i:i + 128]
        frame = encode_tunnel(chunk,
                              target_sys=args.sys, target_comp=args.comp,
                              src_sys=args.src_sys, src_comp=args.src_comp,
                              seq=seq)
        sock.sendto(frame, (args.host, args.port))
        sent += 1
        seq = (seq + 1) % 256
        time.sleep(0.01)
    print(f"sent {sent} TUNNEL frame(s), total {len(payload)} payload bytes "
          f"→ {args.host}:{args.port}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
