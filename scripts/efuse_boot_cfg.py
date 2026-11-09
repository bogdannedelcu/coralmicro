#!/usr/bin/env python3
"""Read (and, only with --burn, program) the RT1176 boot eFuses so that
BOOT_MODE[1:0] = 00 ("boot from fuses") boots the serial NAND.

Board rev 1.1 has no pull-up on BOOT_MODE1, so at power-on the ROM reads
BOOT_MODE = 00 and, with BT_FUSE_SEL = 0, falls back to the Serial Downloader
(USB 1fc9:013d) instead of booting the flashed firmware.  Programming
BOOT_CFG (0x940) + BT_FUSE_SEL (0x960) makes the ROM boot the NAND directly;
see docs/board_rev_1_1/boot_efuses.md and boot_straps_probe.md.  Recovery
into the Serial Downloader stays available by holding the USER/BOOT button
at reset.

Reuses flashtool's machinery: a running firmware is reset into ROM SDP over
HID, the flashloader is loaded, then blhost efuse-read-once / efuse-program-once.

Words (IMXRT1170RM Rev.3, Table 26-6 "FlexSPI (Serial NAND) boot fusemap" and
Table 10-11; blhost index = (addr - 0x800) >> 4):
  0x940 (idx 0x14)  BOOT_CFG1[7:0] | BOOT_CFG2[15:8]
                    target 0x000004C0 = BOOT_CFG1 0xC0 (bits 7:6 = 11 Serial NAND,
                    50 MHz, 12-bit column, hold by read-status, stride 64 pages),
                    BOOT_CFG2 0x04 (bits 2:1 = 10 -> CS interval 400 ns, 1 search,
                    FlexSPI1) -- exactly SRC_SBMR1 of the strap-booting boards.
  0x960 (idx 0x16)  bit 4 BT_FUSE_SEL -> 0x00000010
  0x9A0 (idx 0x1A)  FLASH_CONNECTION_SEL / pin group -- leave 0 (PORTA, primary)
  0xC80 (idx 0x48)  SPI NAND timing overrides -- leave 0
Fuses are OR-only and permanent.  Never touch SDP_DISABLE / SEC_CONFIG / DIR_BT_DIS.

Usage:
  python3 efuse_boot_cfg.py --read                # non-destructive
  python3 efuse_boot_cfg.py --burn I-UNDERSTAND   # programs 0x940 then 0x960, with read-back
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))   # repo root (this file lives in scripts/)
sys.path.insert(0, HERE)
import flashtool as ft  # noqa: E402

BLHOST = os.path.join(ROOT, 'third_party', 'nxp', 'blhost', 'bin', 'linux', 'amd64', 'blhost')
WORDS = {0x14: 'FUSE 0x940 BOOT_CFG1/2', 0x16: 'FUSE 0x960 BT_FUSE_SEL(b4)', 0x1A: 'FUSE 0x9A0 FLASH_CONN/PINGRP', 0x48: 'FUSE 0xC80 NAND timing'}
TARGET = {0x14: 0x000004C0, 0x16: 0x00000010}


def bl(*args, check=True):
    cmd = [BLHOST, '-u', ft.flashloader_vidpid(), '--'] + list(args)
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if check and p.returncode != 0:
        raise RuntimeError('blhost %s failed: %s %s' % (args, p.stdout[-300:], p.stderr[-300:]))
    return p.stdout


def read_once(idx):
    out = bl('efuse-read-once', hex(idx))
    # blhost prints "Response word 1 = 4 (0x4)" then "Response word 2 = <value> (0x...)"
    vals = [l for l in out.splitlines() if 'Response word 2' in l]
    if not vals:
        raise RuntimeError('no value in: ' + out[-300:])
    return int(vals[0].split('(')[1].rstrip(')'), 16)


def enter_flashloader():
    if ft.is_flashloader_connected():
        return
    if not ft.is_sdp_connected():
        # Same trick flashtool uses: open the board's CDC port at 1200 baud with
        # DTR low -> the coralmicro USB stack resets into ROM SDP.  Find the port
        # by VID/PID (flashtool matches on the USB serial number instead).
        import serial, serial.tools.list_ports
        port = None
        for pinfo in serial.tools.list_ports.comports():
            if pinfo.vid == ft.SDP_VID and pinfo.pid == 0xC0A1:
                port = pinfo.device
        if port is None:
            raise RuntimeError('no CDC port of a running coralmicro firmware found (1fc9:c0a1)')
        print('resetting running firmware on %s into SDP (1200 baud / DTR) ...' % port)
        try:
            with serial.Serial(baudrate=1200) as sp:
                sp.port = port; sp.dtr = False; sp.open(); sp.write(b'42')
        except (serial.SerialException, OSError):
            pass   # the port vanishes as the board resets into SDP
        for _ in range(40):
            if ft.is_sdp_connected(): break
            time.sleep(0.25)
    if not ft.is_sdp_connected():
        raise RuntimeError('board not in SDP (1fc9:013d)')
    build_dir = os.path.join(ROOT, 'build')
    elftosb = os.path.join(ROOT, 'third_party', 'nxp', 'elftosb', 'linux', 'amd64', 'elftosb')
    workdir, fl_path = ft.MakeFlashloader(build_dir, None, elftosb)
    print('loading flashloader ...')
    ft.StateLoadFlashloader(BLHOST, fl_path)
    for _ in range(40):
        if ft.is_flashloader_connected(): break
        time.sleep(0.25)
    if not ft.is_flashloader_connected():
        raise RuntimeError('flashloader did not enumerate (15a2:0073)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--read', action='store_true')
    ap.add_argument('--burn', default='', help='pass I-UNDERSTAND to program the two words')
    a = ap.parse_args()
    enter_flashloader()
    vals = {}
    for idx, name in WORDS.items():
        vals[idx] = read_once(idx)
        print('%-30s idx 0x%02X = 0x%08X' % (name, idx, vals[idx]))
    if a.burn != 'I-UNDERSTAND':
        print('read-only run (use --burn I-UNDERSTAND to program).')
        return 0
    for idx in (0x14, 0x16):
        want = TARGET[idx]
        if (vals[idx] & want) == want:
            print('idx 0x%02X already has 0x%08X set, skipping' % (idx, want)); continue
        print('PROGRAMMING idx 0x%02X |= 0x%08X ...' % (idx, want))
        bl('efuse-program-once', hex(idx), '%08X' % want)
        time.sleep(0.2)
        rb = read_once(idx)
        print('  read-back 0x%08X %s' % (rb, 'OK' if (rb & want) == want else '*** MISMATCH ***'))
        if (rb & want) != want:
            return 2
    print('done; resetting the board')
    bl('reset', check=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
