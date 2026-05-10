#!/usr/bin/env python3
"""Read drone-side bridge PARAM counters via Crazyradio PA.

Locates the failure point in the radio→UART→board→UART→radio chain:

    R2U                = packets the drone forwarded radio→UART (board-bound)
    U2R                = packets the drone forwarded UART→radio (board responses)
    Ucrc / Ubad        = UART RX errors on the drone (CRC / bad len)
    Flow / FlowDrp     = optical flow injects (board → drone EKF)

Interpretation:
    R2U=0          → drone deck driver not loaded or radio not delivering
    R2U>0, U2R=0   → board not responding (board hung, build w/o auto-init,
                     UART wiring broken, or bridge dispatcher stalled)
    Ucrc>0/Ubad>0  → board IS sending bytes, but framing is mangled
    Flow=0 mid-run → board's send_flow path not active
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M/E7E7E7E7E7'

logging.basicConfig(level=logging.ERROR)


def main():
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print(f'connecting to {URI} ...')
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='/tmp/cf_cache')) as scf:
        cf = scf.cf
        time.sleep(0.5)

        keys = (
            'deck.sentaiR2U', 'deck.sentaiR2Udrp',
            'deck.sentaiU2R', 'deck.sentaiU2Rdrp',
            'deck.sentaiUcrc', 'deck.sentaiUbad',
            'deck.sentaiFlow', 'deck.sentaiFlowDrp',
        )
        print('-- bridge counters (PARAM cache; reconnect to refresh) --')
        for k in keys:
            try:
                print(f'  {k:28s} = {cf.param.get_value(k)}')
            except Exception as e:
                print(f'  {k:28s} ERR {e}')


if __name__ == '__main__':
    main()
