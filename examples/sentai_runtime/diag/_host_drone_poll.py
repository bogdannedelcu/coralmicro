"""_host_drone_poll.py -- background poller for drone PARAM counters.

Runs in parallel with an on-board flow inject test; samples
deck.sentai{Flow,FlBad,Ucrc,U2R} via cflib over the Crazyradio
every <interval> seconds for <duration> seconds, writes one CSV row
per sample to stdout.

Usage:
    /path/to/cf-venv/bin/python diag/_host_drone_poll.py \\
        --uri radio://0/80/2M --duration 30 --interval 0.5

Output:
    t_s,sentaiFlow,sentaiFlBad,sentaiUcrc,sentaiU2R
    0.00,180,0,0,0
    0.50,195,0,0,0
    ...
"""
import argparse
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='radio://0/80/2M')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--interval', type=float, default=0.5)
    args = ap.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    print('t_s,sentaiFlow,sentaiFlBad,sentaiUcrc,sentaiU2R', flush=True)
    with SyncCrazyflie(args.uri,
                       cf=Crazyflie(rw_cache='/tmp/cf_cache')) as scf:
        cf = scf.cf
        time.sleep(0.3)   # let TOC settle
        t_start = time.time()
        while True:
            t = time.time() - t_start
            if t >= args.duration:
                break
            try:
                flow  = cf.param.get_value('deck.sentaiFlow')
                flbad = cf.param.get_value('deck.sentaiFlBad')
                ucrc  = cf.param.get_value('deck.sentaiUcrc')
                u2r   = cf.param.get_value('deck.sentaiU2R')
                print('%.2f,%s,%s,%s,%s' %
                      (t, flow, flbad, ucrc, u2r), flush=True)
            except Exception as e:
                print('# err t=%.2f: %r' % (t, e), flush=True)
            time.sleep(args.interval)


if __name__ == '__main__':
    main()
