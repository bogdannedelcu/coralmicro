#!/usr/bin/env python3
"""Capture the USB CDC-ACM console of a vendor example loaded with
`flashtool.py --ram` into the board.

The RAM-loaded vendor examples (branch lucian/board-rev-1.1) enumerate as
18d1:9308 and print on /dev/ttyACM0.  The port is put in raw mode with stty
and read with a blocking `cat` for `--seconds` into `--out` (pyserial and a
non-blocking reader both lost data on this console -- iter1).

Usage: python3 capture_console.py --out iter1/sdcard_test.log --seconds 45
"""
import argparse, glob, os, subprocess, sys, time


def wait_port(pattern, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
        time.sleep(0.2)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM*')
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=45)
    ap.add_argument('--wait', type=float, default=30, help='max wait for the port')
    a = ap.parse_args()

    port = wait_port(a.port, a.wait)
    if not port:
        print('no console port within %.0f s' % a.wait, file=sys.stderr)
        sys.exit(2)
    time.sleep(0.3)
    subprocess.run(['stty', '-F', port, 'raw', '115200', '-echo', '-echoe', '-echok', '-hupcl'],
                   check=False)
    print('capturing %s for %.0f s -> %s' % (port, a.seconds, a.out))
    # A plain blocking reader (cat) is what reliably received every byte on
    # this console; a non-blocking os.read() loop and pyserial both missed data.
    with open(a.out, 'wb') as f:
        subprocess.run(['timeout', str(a.seconds), 'cat', port], stdout=f, check=False)
    n = os.path.getsize(a.out)
    print('captured %d bytes' % n)


if __name__ == '__main__':
    main()
