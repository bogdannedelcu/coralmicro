#!/usr/bin/env python3
"""End-to-end radio smoke test — board on drone battery, USB unplugged.

Sends `$1+1` via Crazyradio PA → drone CRTP port 0x0E → deck driver
forwards to UART2 → SentAI board's `sentai_crazy.cc` 0xAA parser →
MicroPython `crazy_run_exec` evaluates "1+1" → reply `OK 2` ships back
the same path. Expected reply: a single fragment `b'\x00OK 2'`
(0x00 is the MF=0 last-fragment marker on CH=0).

Pass criteria: `OK 2` substring in joined reply within TIMEOUT_S.

Builds the bridge needs: board ≥ #1223 (firmware auto-init of crazy
bridge); drone fork `bogdannedelcu/crazyflie-firmware` branch
`sentai-deck-driver` with `CONFIG_ENABLE_CPX=n`.
"""
import logging
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket

URI = 'radio://0/80/2M/E7E7E7E7E7'
SENTAI_PORT = 0x0E
TIMEOUT_S = 5.0

logging.basicConfig(level=logging.ERROR)


def main():
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print(f'connecting to {URI} ...')

    rx_data = []
    rx_event = threading.Event()

    def on_packet(pkt):
        rx_data.append(bytes(pkt.data))
        rx_event.set()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='/tmp/cf_cache')) as scf:
        cf = scf.cf
        cf.add_port_callback(SENTAI_PORT, on_packet)
        time.sleep(0.3)

        pk = CRTPPacket()
        pk.set_header(SENTAI_PORT, 0)  # channel 0 = REPL/exec
        pk.data = b'$1+1'
        print(f'sending port=0x{SENTAI_PORT:02X} ch=0 data={pk.data!r}')
        cf.send_packet(pk)

        if not rx_event.wait(TIMEOUT_S):
            print(f'FAIL: no reply within {TIMEOUT_S}s')
            return 1

        time.sleep(0.2)  # collect any tail fragment
        joined = b''.join(rx_data)
        print(f'got {len(rx_data)} fragment(s) totalling {len(joined)} bytes:')
        for i, frag in enumerate(rx_data):
            print(f'  [{i}] {frag!r}')

        if b'OK 2' in joined:
            print('PASS: $1+1 -> OK 2 via radio')
            return 0
        print('FAIL: expected OK 2 in reply')
        return 1


if __name__ == '__main__':
    sys.exit(main())
