#!/usr/bin/env python3
# upload_diag.py — upload diag/ package to device via HTTP
#
# Prerequisites: sentai.usb.ip(1) must be active (USB-ETH CDC-NCM).
#
# Usage:
#   python3 upload_diag.py                    # assumes device at 10.0.0.1
#   python3 upload_diag.py --ip 10.0.0.1
#   python3 upload_diag.py --enable-usb       # auto-enable via REPL first
#   python3 upload_diag.py --port /dev/ttyACM1 --enable-usb
#   python3 upload_diag.py --file e_tpu.py    # upload only one file
#   python3 upload_diag.py --browser          # push browser.html to /.sys/

import os
import sys
import time
import argparse
import pathlib
import urllib.request
import urllib.error
import json

DIAG_DIR = pathlib.Path(__file__).parent / "diag"
BROWSER_SRC = pathlib.Path(__file__).parent / "web" / "browser.html"
DEFAULT_IP = "10.0.0.1"
REMOTE_BASE = "/lib/diag"
BROWSER_REMOTE = "/.sys/browser.html"


def http_post(ip, remote_path, data, timeout=15):
    """POST raw bytes to /api/write/<remote_path>. Returns parsed JSON."""
    url = "http://%s/api/write%s" % (ip, remote_path)
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/octet-stream")
    req.add_header("Content-Length", str(len(data)))
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read().decode()
    return json.loads(body)


def enable_usb_ip(port, baud=115200):
    """Send sentai.usb.ip(1) via REPL and wait for USB-ETH to come up."""
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    print("Enabling USB IP via REPL on %s ..." % port)
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(0.3)
    ser.read(ser.in_waiting or 1)
    ser.write(b"\x03\r\n")
    time.sleep(0.3)
    ser.read(ser.in_waiting or 1)
    ser.write(b"sentai.usb.ip(1)\r\n")
    time.sleep(2.5)
    out = ser.read(ser.in_waiting or 1).decode(errors="replace")
    ser.close()

    if "10.0.0.1" in out or "active" in out.lower() or "CDC" in out:
        print("  USB IP active.")
        return True
    print("  WARNING: could not confirm USB IP from REPL output.")
    print("  Output was: %s" % repr(out[-200:]))
    return False


def wait_for_device(ip, retries=10, delay=1.0):
    """Poll GET / until the device responds."""
    url = "http://%s/" % ip
    for i in range(retries):
        try:
            urllib.request.urlopen(url, timeout=3)
            return True
        except Exception:
            if i == 0:
                print("  Waiting for device at %s ..." % ip, end="", flush=True)
            else:
                print(".", end="", flush=True)
            time.sleep(delay)
    print()
    return False


def upload_file(ip, local_path, remote_path):
    data = local_path.read_bytes()
    result = http_post(ip, remote_path, data)
    return result, len(data)


def main():
    parser = argparse.ArgumentParser(
        description="Upload diag/ package to SentAI device via HTTP")
    parser.add_argument("--ip",         default=DEFAULT_IP,
                        help="Device IP (default: %(default)s)")
    parser.add_argument("--port",       default="/dev/ttyACM0",
                        help="Serial port for --enable-usb (default: %(default)s)")
    parser.add_argument("--enable-usb", action="store_true",
                        help="Enable USB IP via REPL before uploading")
    parser.add_argument("--file",       default=None,
                        help="Upload only this filename (e.g. e_tpu.py)")
    parser.add_argument("--no-wait",    action="store_true",
                        help="Skip device reachability check")
    parser.add_argument("--browser",    action="store_true",
                        help="Push browser.html to /.sys/browser.html on device")
    args = parser.parse_args()

    if args.browser:
        if not BROWSER_SRC.exists():
            print("ERROR: %s does not exist" % BROWSER_SRC)
            sys.exit(1)
        if not args.no_wait:
            if not wait_for_device(args.ip):
                print("\nERROR: device not reachable at http://%s/" % args.ip)
                sys.exit(1)
            print()
        data = BROWSER_SRC.read_bytes()
        try:
            result = http_post(args.ip, BROWSER_REMOTE, data)
            if result.get("ok"):
                print("OK  browser.html → %s  (%d bytes)" % (BROWSER_REMOTE, len(data)))
            else:
                print("FAIL: %s" % result.get("error", "?"))
                sys.exit(1)
        except Exception as e:
            print("ERR: %s" % e)
            sys.exit(1)
        return

    if not DIAG_DIR.is_dir():
        print("ERROR: %s does not exist" % DIAG_DIR)
        sys.exit(1)

    if args.enable_usb:
        ok = enable_usb_ip(args.port)
        time.sleep(1.5)  # give NCM stack a moment

    if not args.no_wait:
        if not wait_for_device(args.ip):
            print("\nERROR: device not reachable at http://%s/" % args.ip)
            print("  Make sure USB is connected and sentai.usb.ip(1) is active.")
            sys.exit(1)
        print()  # newline after dots

    # Collect files to upload
    if args.file:
        files = [DIAG_DIR / args.file]
        if not files[0].exists():
            print("ERROR: %s not found in %s" % (args.file, DIAG_DIR))
            sys.exit(1)
    else:
        files = sorted(DIAG_DIR.glob("*.py"))

    if not files:
        print("ERROR: no .py files found in %s" % DIAG_DIR)
        sys.exit(1)

    print("Uploading %d file(s) to http://%s%s/" % (
        len(files), args.ip, REMOTE_BASE))
    print()

    errors = 0
    total_bytes = 0

    for f in files:
        remote = "%s/%s" % (REMOTE_BASE, f.name)
        try:
            result, size = upload_file(args.ip, f, remote)
            total_bytes += size
            if result.get("ok"):
                print("  OK   %-30s  %d bytes" % (f.name, size))
            else:
                print("  FAIL %-30s  %s" % (f.name, result.get("error", "?")))
                errors += 1
        except urllib.error.URLError as e:
            print("  ERR  %-30s  %s" % (f.name, e))
            errors += 1
        except Exception as e:
            print("  ERR  %-30s  %s" % (f.name, e))
            errors += 1

    print()
    if errors:
        print("%d error(s). Check USB connection and USB IP status." % errors)
        sys.exit(1)
    else:
        print("Done. %d files, %d bytes total." % (len(files), total_bytes))
        print("On device:  import diag")
        print("            diag.begin('test')")
        print("            diag.e1_tpu_invoke('/yolo26n.edgetpu_1.tflite')")


if __name__ == "__main__":
    main()
