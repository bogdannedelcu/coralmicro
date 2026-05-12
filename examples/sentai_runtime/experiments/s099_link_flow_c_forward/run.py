#!/usr/bin/env python3
"""s099 — verify C-side flow forwarder.

Plumbing:
  +-------------------+  /tmp/sentai_cam.sock  +-------------------+
  | synthetic feeder  | ---------------------> |    sentai_sim     |
  | (this script)     |                        |  camera_bridge    |
  +-------------------+                        |  + flow_phase_corr|
                                               |  + link_fwd task  |
                                               +--------+----------+
                                                        | UDP :14580
                                                        v
                                               +-------------------+
                                               | sniffer (this scr)|
                                               +-------------------+

Pass: ≥10 OPTICAL_FLOW_RAD (msgid 106) frames seen, stats[8] matches.
"""
from __future__ import annotations
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
SIM_BIN = REPO / "build-sim/sim/sentai_sim"
CAM_SOCK = "/tmp/sentai_cam.sock"

MAGIC      = 0x53434D31  # 'SCM1'
HEADER_FMT = "<IIIIII"
W, H = 640, 480

# MAVLink v2 frame: byte 0 = 0xFD (magic), msgid at offset 7..9 (LE 24-bit).
MAV2_STX = 0xFD
OPTICAL_FLOW_RAD_MSGID = 106


def synth_rgb(seq: int) -> bytes:
    # Cheap deterministic content with a slow drift so flow has something
    # to lock onto (constant frame → zero flow → fwd may still send but
    # the counter ticks regardless of magnitude).
    shift = seq % 16
    row = bytes(((shift + x) & 0xFF for x in range(W))) * 3
    # Tile the row into the full frame — same row everywhere (vertical
    # gradient → no flow), then perturb the first 60 rows so phase corr
    # has signal.
    full = bytearray(row * H)
    for y in range(60):
        for x in range(0, W * 3, 3):
            full[y * W * 3 + x] = (shift * 13 + y * 7 + x) & 0xFF
    return bytes(full)


def feeder_thread(stop: threading.Event, n_sent_box: list[int]) -> None:
    # Connect with retries — sim is still booting.
    s = None
    for _ in range(60):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(CAM_SOCK)
            break
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.1)
    if s is None:
        print("[feeder] could not connect to", CAM_SOCK, file=sys.stderr)
        return
    print(f"[feeder] connected to {CAM_SOCK}")
    seq = 1
    while not stop.is_set():
        rgb = synth_rgb(seq)
        hdr = struct.pack(HEADER_FMT, MAGIC, seq, W, H, 0, len(rgb))
        try:
            s.sendall(hdr + rgb)
        except (BrokenPipeError, ConnectionResetError):
            break
        # Drain any reply (REPLY_MAGIC frames) to keep socket healthy.
        try:
            s.setblocking(False)
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
        except BlockingIOError:
            pass
        finally:
            s.setblocking(True)
        seq += 1
        n_sent_box[0] = seq - 1
        time.sleep(0.033)  # 30 Hz
    s.close()
    print(f"[feeder] stopped (sent {seq-1} frames)")


def sniffer_thread(stop: threading.Event, count_box: list[int]) -> None:
    """Listen on the PX4-side port (14580) — sim sends here."""
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("127.0.0.1", 14580))
    udp.settimeout(0.2)
    print("[sniffer] listening on :14580")
    n_flow = 0
    while not stop.is_set():
        try:
            data, _ = udp.recvfrom(2048)
        except socket.timeout:
            continue
        # Walk through bytes; MAVLink v2 framing.
        i = 0
        while i < len(data):
            if data[i] != MAV2_STX:
                i += 1
                continue
            if i + 10 >= len(data):
                break
            payload_len = data[i + 1]
            msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            frame_total = 12 + payload_len + 2  # header + payload + CRC
            if msgid == OPTICAL_FLOW_RAD_MSGID:
                n_flow += 1
                count_box[0] = n_flow
            i += frame_total
    udp.close()
    print(f"[sniffer] stopped (flow={n_flow})")


def main() -> int:
    if not SIM_BIN.exists():
        print("ERROR: build first — cmake --build build-sim --target sentai_sim",
              file=sys.stderr)
        return 2

    # Clean any stale sentai_sim — port collision is fatal.
    subprocess.run(["pkill", "-f", "sentai_sim"], check=False)
    time.sleep(0.3)

    print("[s099] spawning sentai_sim")
    proc = subprocess.Popen(
        [str(SIM_BIN)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        env=os.environ.copy(),
    )

    stop = threading.Event()
    sent_box = [0]
    count_box = [0]

    # Drain sim output in background so the buffer never blocks.
    drain_lines: list[bytes] = []

    def drain():
        for line in proc.stdout:
            drain_lines.append(line)

    threading.Thread(target=drain, daemon=True).start()

    feeder = threading.Thread(target=feeder_thread, args=(stop, sent_box))
    sniffer = threading.Thread(target=sniffer_thread, args=(stop, count_box))
    feeder.start()
    sniffer.start()

    # Wait a moment for sim init + socket listen.
    time.sleep(1.0)

    # Drive the REPL: init link, start forwarder, hold for measurement, stop.
    script = (
        b"import sentai\n"
        b"print('INIT:', sentai.link.init(57600, 1, 191))\n"
        b"sentai.rtos.sleep_ms(200)\n"
        b"print('FLOW1:', sentai.link.flow(1, 1.0))\n"
        b"sentai.rtos.sleep_ms(2000)\n"
        b"print('STATS:', sentai.link.stats())\n"
        b"print('FLOW0:', sentai.link.flow(0))\n"
        b"sentai.rtos.sleep_ms(200)\n"
        b"print('STATS_END:', sentai.link.stats())\n"
    )
    proc.stdin.write(script)
    proc.stdin.flush()

    # Hold for the script + drain.
    time.sleep(3.5)
    stop.set()
    proc.stdin.close()
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        proc.wait(timeout=2.0)

    feeder.join(timeout=1.0)
    sniffer.join(timeout=1.0)

    output = b"".join(drain_lines).decode("utf-8", errors="replace")
    sent = sent_box[0]
    sniffed = count_box[0]
    # Parse STATS line for tx_flow (index 8 in the tuple).
    tx_flow = None
    for line in output.splitlines():
        # Sim prints REPL output with leading `>>> ` prompt fragments.
        if "STATS:" in line and "STATS_END" not in line:
            try:
                tup = line.split("STATS:", 1)[1].strip()
                # tup looks like "(0, 0, 0, 0, 0, 0, 0, 0, 32)"
                tup = tup.strip("()")
                parts = [p.strip() for p in tup.split(",")]
                tx_flow = int(parts[8])
            except (IndexError, ValueError):
                pass

    print()
    print("=== s099 results ===")
    print(f"feeder frames sent:       {sent}")
    print(f"sniffer OPTICAL_FLOW_RAD: {sniffed}")
    print(f"sim stats[8] (tx_flow):   {tx_flow}")
    print()
    print("--- sim output (tail) ---")
    print(output[-1500:] if len(output) > 1500 else output)

    pass_min = 10
    ok = (sniffed >= pass_min) and (tx_flow is not None) and (abs(tx_flow - sniffed) <= 1)
    print()
    print("PASS" if ok else "FAIL", f"(criterion: sniffed>={pass_min}, tx_flow≈sniffed)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
