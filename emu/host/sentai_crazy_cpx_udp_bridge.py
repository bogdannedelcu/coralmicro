#!/usr/bin/env python3
"""Host CPX UART <-> CrazySim CRTP UDP bridge for the ARM emulator.

The guest keeps using the shared `sentai.crazy` runtime path:

    sentai.crazy -> CRTP -> CPX -> UART

This host process terminates the emulator UART stream, acknowledges CPX CTS
flow control, forwards CRTP payloads to cf2 UDP port 19850, and wraps inbound
CRTP datagrams back into CPX frames for the guest.
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import select
import socket
import sys
import time
from typing import Iterable


CPX_START_BYTE = 0xFF
CPX_MTU = 100
CPX_CTS = bytes([CPX_START_BYTE, 0x00])

CPX_T_STM32 = 1
CPX_T_ESP32 = 2
CPX_T_HOST = 3

CPX_F_SYSTEM = 1
CPX_F_CONSOLE = 2
CPX_F_CRTP = 3
CPX_F_WIFI_CTRL = 4
CPX_F_APP = 5

CRTP_MAX_PAYLOAD = 30
CRTP_MAX_DATAGRAM = 1 + CRTP_MAX_PAYLOAD


@dataclasses.dataclass(frozen=True)
class CpxEvent:
    kind: str
    data: bytes = b""
    dst: int = 0
    src: int = 0
    fn: int = 0
    payload: bytes = b""
    message: str = ""


@dataclasses.dataclass
class BridgeStats:
    cpx_frames_rx: int = 0
    cpx_cts_rx: int = 0
    cpx_bad_frames: int = 0
    cpx_system_rx: int = 0
    cpx_console_rx: int = 0
    crtp_to_udp: int = 0
    crtp_from_udp: int = 0
    serial_bytes_rx: int = 0
    serial_bytes_tx: int = 0
    udp_bytes_rx: int = 0
    udp_bytes_tx: int = 0


def cpx_crc(data: bytes | bytearray | memoryview) -> int:
    crc = 0
    for byte in data:
        crc ^= int(byte)
    return crc & 0xFF


def pack_route(dst: int, src: int, fn: int) -> bytes:
    return bytes([
        (1 << 6) | ((src & 0x07) << 3) | (dst & 0x07),
        fn & 0x3F,
    ])


def unpack_route(data: bytes) -> tuple[int, int, int]:
    if len(data) < 2:
        raise ValueError("CPX data needs a 2-byte route header")
    dst = data[0] & 0x07
    src = (data[0] >> 3) & 0x07
    fn = data[1] & 0x3F
    return dst, src, fn


def build_cpx_frame(cpx_data: bytes) -> bytes:
    if len(cpx_data) < 2 or len(cpx_data) > CPX_MTU:
        raise ValueError(f"invalid CPX payload length {len(cpx_data)}")
    frame = bytearray()
    frame.append(CPX_START_BYTE)
    frame.append(len(cpx_data))
    frame.extend(cpx_data)
    frame.append(cpx_crc(frame))
    return bytes(frame)


def build_cpx_packet(dst: int, src: int, fn: int, payload: bytes = b"") -> bytes:
    return build_cpx_frame(pack_route(dst, src, fn) + bytes(payload))


def build_crtp_to_cf2(crtp_datagram: bytes) -> bytes:
    if len(crtp_datagram) > CRTP_MAX_DATAGRAM:
        raise ValueError(f"CRTP datagram too large: {len(crtp_datagram)}")
    return build_cpx_packet(CPX_T_STM32, CPX_T_HOST, CPX_F_CRTP, crtp_datagram)


def build_crtp_to_guest(crtp_datagram: bytes) -> bytes:
    if len(crtp_datagram) > CRTP_MAX_DATAGRAM:
        raise ValueError(f"CRTP datagram too large: {len(crtp_datagram)}")
    return build_cpx_packet(CPX_T_HOST, CPX_T_STM32, CPX_F_CRTP, crtp_datagram)


def parse_cpx_stream(buffer: bytes) -> tuple[list[CpxEvent], bytes]:
    events: list[CpxEvent] = []
    cursor = 0
    n = len(buffer)

    while cursor < n:
        start = buffer.find(bytes([CPX_START_BYTE]), cursor)
        if start < 0:
            return events, b""
        cursor = start

        if n - cursor < 2:
            return events, buffer[cursor:]

        cpx_len = buffer[cursor + 1]
        if cpx_len == 0:
            events.append(CpxEvent(kind="cts"))
            cursor += 2
            continue

        if cpx_len > CPX_MTU:
            events.append(CpxEvent(
                kind="bad",
                message=f"invalid CPX length {cpx_len}",
            ))
            cursor += 1
            continue

        total_len = 2 + cpx_len + 1
        if n - cursor < total_len:
            return events, buffer[cursor:]

        frame = buffer[cursor:cursor + total_len]
        expected = cpx_crc(frame[:-1])
        got = frame[-1]
        if got != expected:
            events.append(CpxEvent(
                kind="bad",
                message=f"bad CPX CRC got=0x{got:02x} expected=0x{expected:02x}",
            ))
            cursor += 1
            continue

        data = frame[2:2 + cpx_len]
        try:
            dst, src, fn = unpack_route(data)
        except ValueError as exc:
            events.append(CpxEvent(kind="bad", data=data, message=str(exc)))
        else:
            events.append(CpxEvent(
                kind="frame",
                data=data,
                dst=dst,
                src=src,
                fn=fn,
                payload=data[2:],
            ))

        cursor += total_len

    return events, b""


def _write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        written = os.write(fd, view)
        view = view[written:]


def _open_serial(path: str) -> tuple[int, int, bool]:
    if path == "-":
        return sys.stdin.buffer.fileno(), sys.stdout.buffer.fileno(), False
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd, fd, True


def _log(verbose: bool, msg: str) -> None:
    if verbose:
        print(msg, file=sys.stderr, flush=True)


def _log_stats(stats: BridgeStats, started: float) -> None:
    elapsed = max(time.monotonic() - started, 0.001)
    print(
        "stats "
        f"elapsed_s={elapsed:.3f} "
        f"cpx_rx={stats.cpx_frames_rx} "
        f"cts_rx={stats.cpx_cts_rx} "
        f"bad={stats.cpx_bad_frames} "
        f"crtp_to_udp={stats.crtp_to_udp} "
        f"crtp_from_udp={stats.crtp_from_udp} "
        f"serial_rx={stats.serial_bytes_rx} "
        f"serial_tx={stats.serial_bytes_tx} "
        f"udp_rx={stats.udp_bytes_rx} "
        f"udp_tx={stats.udp_bytes_tx}",
        file=sys.stderr,
        flush=True,
    )


def run_bridge(serial_path: str,
               udp_host: str,
               udp_port: int,
               verbose: bool = False,
               stats_period_s: float = 5.0) -> int:
    serial_rx, serial_tx, close_serial = _open_serial(serial_path)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    udp.connect((udp_host, udp_port))

    stats = BridgeStats()
    buffer = b""
    started = time.monotonic()
    next_stats = started + stats_period_s

    try:
        _write_all(serial_tx, CPX_CTS)
        stats.serial_bytes_tx += len(CPX_CTS)
        _log(verbose, "sent initial CPX CTS to guest")

        while True:
            read_fds: Iterable[object] = (serial_rx, udp)
            ready, _, _ = select.select(read_fds, (), (), 0.1)

            if serial_rx in ready:
                chunk = os.read(serial_rx, 4096)
                if not chunk:
                    _log(verbose, "serial EOF")
                    return 0
                stats.serial_bytes_rx += len(chunk)
                buffer += chunk
                events, buffer = parse_cpx_stream(buffer)
                for event in events:
                    if event.kind == "cts":
                        stats.cpx_cts_rx += 1
                        continue
                    if event.kind == "bad":
                        stats.cpx_bad_frames += 1
                        _log(verbose, f"bad CPX frame: {event.message}")
                        continue

                    stats.cpx_frames_rx += 1
                    _write_all(serial_tx, CPX_CTS)
                    stats.serial_bytes_tx += len(CPX_CTS)

                    if event.fn == CPX_F_CRTP:
                        udp.send(event.payload)
                        stats.crtp_to_udp += 1
                        stats.udp_bytes_tx += len(event.payload)
                        _log(verbose, f"guest->cf2 crtp_len={len(event.payload)}")
                    elif event.fn == CPX_F_SYSTEM:
                        stats.cpx_system_rx += 1
                        _log(verbose, f"guest system payload={event.payload.hex()}")
                    elif event.fn == CPX_F_CONSOLE:
                        stats.cpx_console_rx += 1
                        _log(verbose, f"guest console payload={event.payload!r}")
                    else:
                        _log(verbose, f"ignored CPX fn={event.fn} len={len(event.payload)}")

            if udp in ready:
                crtp = udp.recv(256)
                if len(crtp) > CRTP_MAX_DATAGRAM:
                    stats.cpx_bad_frames += 1
                    _log(verbose, f"dropped oversized cf2 CRTP datagram len={len(crtp)}")
                else:
                    frame = build_crtp_to_guest(crtp)
                    _write_all(serial_tx, frame)
                    stats.crtp_from_udp += 1
                    stats.udp_bytes_rx += len(crtp)
                    stats.serial_bytes_tx += len(frame)
                    _log(verbose, f"cf2->guest crtp_len={len(crtp)}")

            now = time.monotonic()
            if stats_period_s > 0 and now >= next_stats:
                _log_stats(stats, started)
                next_stats = now + stats_period_s

    finally:
        udp.close()
        if close_serial:
            os.close(serial_rx)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Bridge SentAI emulator CPX UART to CrazySim CRTP UDP.")
    parser.add_argument(
        "--serial",
        default="-",
        help="UART byte stream path, or '-' for stdin/stdout.",
    )
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=19850)
    parser.add_argument("--stats-period-s", type=float, default=5.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    return run_bridge(
        serial_path=args.serial,
        udp_host=args.udp_host,
        udp_port=args.udp_port,
        verbose=args.verbose,
        stats_period_s=args.stats_period_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
