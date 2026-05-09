"""_host_drone_log.py -- subscribe to kalman_pred LOG block, write CSV.

Push-based (vs PARAM get_value which is cached).  Streams the drone
EKF's predicted vs measured flow pixel motion at 100 ms cadence.

Usage:
    /path/to/cf-venv/bin/python diag/_host_drone_log.py \\
        --uri radio://0/80/2M --duration 35
"""
import argparse
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='radio://0/80/2M')
    ap.add_argument('--duration', type=float, default=35.0)
    ap.add_argument('--period_ms', type=int, default=100)
    args = ap.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    print('t_s,predNX,predNY,measNX,measNY', flush=True)

    with SyncCrazyflie(args.uri,
                       cf=Crazyflie(rw_cache='/tmp/cf_cache')) as scf:
        cf = scf.cf
        time.sleep(0.3)

        lg = LogConfig(name='kp', period_in_ms=args.period_ms)
        for var in ('kalman_pred.predNX', 'kalman_pred.predNY',
                    'kalman_pred.measNX', 'kalman_pred.measNY'):
            lg.add_variable(var, 'float')

        cf.log.add_config(lg)

        t0 = time.time()
        def on_data(timestamp, data, logconf):
            print('%.3f,%.4f,%.4f,%.4f,%.4f' % (
                time.time() - t0,
                data['kalman_pred.predNX'], data['kalman_pred.predNY'],
                data['kalman_pred.measNX'], data['kalman_pred.measNY']),
                flush=True)

        lg.data_received_cb.add_callback(on_data)
        lg.start()
        try:
            while time.time() - t0 < args.duration:
                time.sleep(0.05)
        finally:
            lg.stop()


if __name__ == '__main__':
    main()
