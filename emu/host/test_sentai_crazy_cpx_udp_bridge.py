#!/usr/bin/env python3
"""Smoke tests for sentai_crazy_cpx_udp_bridge.py."""

from __future__ import annotations

import pathlib
import sys


THIS_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

import sentai_crazy_cpx_udp_bridge as bridge  # noqa: E402


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def test_crc_and_route() -> None:
    route = bridge.pack_route(bridge.CPX_T_STM32,
                              bridge.CPX_T_HOST,
                              bridge.CPX_F_CRTP)
    dst, src, fn = bridge.unpack_route(route)
    require(dst == bridge.CPX_T_STM32, "route dst")
    require(src == bridge.CPX_T_HOST, "route src")
    require(fn == bridge.CPX_F_CRTP, "route fn")

    frame = bridge.build_cpx_frame(route + b"\x30\x01\x02")
    require(frame[0] == bridge.CPX_START_BYTE, "frame start")
    require(frame[1] == 5, "frame len")
    require(frame[-1] == bridge.cpx_crc(frame[:-1]), "frame crc")


def test_guest_to_cf2_crtp_frame() -> None:
    crtp = bytes([0x93, 0x01, 0x02, 0x03])
    frame = bridge.build_crtp_to_cf2(crtp)
    events, rest = bridge.parse_cpx_stream(frame)
    require(rest == b"", "unexpected parse remainder")
    require(len(events) == 1, "expected one event")
    event = events[0]
    require(event.kind == "frame", "expected frame event")
    require(event.dst == bridge.CPX_T_STM32, "guest->cf2 dst")
    require(event.src == bridge.CPX_T_HOST, "guest->cf2 src")
    require(event.fn == bridge.CPX_F_CRTP, "guest->cf2 fn")
    require(event.payload == crtp, "guest->cf2 payload")


def test_cf2_to_guest_crtp_frame() -> None:
    crtp = bytes([0xF3, 0xAA, 0x55])
    frame = bridge.build_crtp_to_guest(crtp)
    events, rest = bridge.parse_cpx_stream(frame)
    require(rest == b"", "unexpected parse remainder")
    require(len(events) == 1, "expected one event")
    event = events[0]
    require(event.kind == "frame", "expected frame event")
    require(event.dst == bridge.CPX_T_HOST, "cf2->guest dst")
    require(event.src == bridge.CPX_T_STM32, "cf2->guest src")
    require(event.fn == bridge.CPX_F_CRTP, "cf2->guest fn")
    require(event.payload == crtp, "cf2->guest payload")


def test_cts_and_partial_parse() -> None:
    frame = bridge.build_crtp_to_cf2(bytes([0x03]))
    events, rest = bridge.parse_cpx_stream(bridge.CPX_CTS + frame[:3])
    require(len(events) == 1, "expected only CTS from partial buffer")
    require(events[0].kind == "cts", "expected CTS event")
    require(rest == frame[:3], "partial frame should remain buffered")

    events2, rest2 = bridge.parse_cpx_stream(rest + frame[3:])
    require(rest2 == b"", "unexpected final remainder")
    require(len(events2) == 1, "expected final frame")
    require(events2[0].kind == "frame", "expected final frame event")


def test_bad_crc_resyncs() -> None:
    good = bridge.build_crtp_to_cf2(bytes([0x73, 0x01]))
    bad = bytearray(good)
    bad[-1] ^= 0x01
    events, rest = bridge.parse_cpx_stream(bytes(bad) + good)
    require(rest == b"", "unexpected remainder after bad+good parse")
    require(len(events) >= 2, "expected bad event and recovered frame")
    require(events[0].kind == "bad", "expected first bad event")
    require(events[-1].kind == "frame", "expected recovery frame")
    require(events[-1].payload == bytes([0x73, 0x01]), "recovered payload")


def main() -> int:
    test_crc_and_route()
    test_guest_to_cf2_crtp_frame()
    test_cf2_to_guest_crtp_frame()
    test_cts_and_partial_parse()
    test_bad_crc_resyncs()
    print("OK sentai_crazy_cpx_udp_bridge")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
