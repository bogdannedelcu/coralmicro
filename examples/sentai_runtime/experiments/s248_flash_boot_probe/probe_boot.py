#!/usr/bin/env python3
"""EXP-s248 — why does the new board rev boot into ROM SDP instead of flash?

Read-only probe through the NXP flashloader (no J-Link, no flash writes):
  1. put the board into ROM SDP (1200-baud touch if firmware is alive),
  2. load the same flashloader flashtool.py uses,
  3. read SRC_SBMR1/SBMR2/SRSR (boot straps, boot mode, reset cause),
  4. configure the FlexSPI NAND exactly like flashtool's .bd file does and
     dump the FCB region + the two boot-image copies' first bytes.

Run from the repo root with the flash venv:
  venv-flash/bin/python examples/sentai_runtime/experiments/s248_flash_boot_probe/probe_boot.py
Afterwards the board is left in the flashloader; re-load firmware with
  venv-flash/bin/python scripts/flashtool.py -e sentai_runtime --ram
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import flashtool  # noqa: E402  (reuse its SDP/flashloader state helpers)

BLHOST = os.path.join(ROOT, 'third_party', 'nxp', 'blhost', 'bin', 'linux', 'amd64', 'blhost')
ELFTOSB = os.path.join(ROOT, 'third_party', 'nxp', 'elftosb', 'linux', 'amd64', 'elftosb')
BUILD = os.path.join(ROOT, 'build')
OUT = HERE

# Option words copied verbatim from flashtool.CreateSbFile() (program_flexspinand_image.bd)
NAND_OPTION_WORDS = [(0x10000, 0xC2000105), (0x10004, 0x00010020),
                     (0x10008, 0x00040004), (0x1000C, 0x00080004),
                     (0x10020, 0xC0010021)]
NAND_MEM_ID = '0x101'


def bl(*args, check=True, quiet=False):
    cmd = [BLHOST, '-u', flashtool.flashloader_vidpid(), '--'] + [str(a) for a in args]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if not quiet:
        print(f'$ blhost {" ".join(str(a) for a in args)}')
        print('  ' + r.stdout.strip().replace('\n', '\n  '))
        if r.stderr.strip():
            print('  [stderr] ' + r.stderr.strip())
    if check and r.returncode != 0:
        raise RuntimeError(f'blhost failed rc={r.returncode}')
    return r.stdout


def read_words(addr, n):
    """read-memory n*4 bytes to a file and return the words."""
    path = os.path.join(OUT, f'mem_{addr:08x}.bin')
    bl('read-memory', hex(addr), n * 4, path, quiet=True)
    data = open(path, 'rb').read()
    os.remove(path)
    return [int.from_bytes(data[i:i + 4], 'little') for i in range(0, len(data), 4)]


def wait_for(pred, what, timeout_s=15):
    for _ in range(timeout_s):
        if pred():
            return True
        time.sleep(1)
    raise RuntimeError(f'timeout waiting for {what}')


def main():
    print('== step 1: board -> SDP')
    if flashtool.is_flashloader_connected():
        print('flashloader already running')
    else:
        if flashtool.is_coral_micro_connected(None) or flashtool.is_elfloader_connected(None):
            r = flashtool.StateResetToSdp()
            if isinstance(r, flashtool.FlashtoolError if hasattr(flashtool, 'FlashtoolError') else object):
                pass
        wait_for(flashtool.is_sdp_connected, 'SDP (1fc9:013d)')
        print('SDP enumerated')
        print('== step 2: load flashloader')
        _wd, fl_path = flashtool.MakeFlashloader(BUILD, None, ELFTOSB)
        flashtool.StateLoadFlashloader(BLHOST, fl_path)
        wait_for(flashtool.is_flashloader_connected, 'flashloader (15a2:0073)')
        time.sleep(1)
    bl('get-property', 1)  # bootloader version — proves the link

    print('== step 3: SRC boot-mode / reset-status registers')
    sbmr1, sbmr2 = read_words(0x40C04008, 2)
    (srsr,) = read_words(0x40C04010, 1)
    bmod = (sbmr2 >> 24) & 0x3
    bt_fuse_sel = (sbmr2 >> 4) & 0x1
    print(f'SRC_SBMR1 = 0x{sbmr1:08X}  BOOT_CFG1=0x{sbmr1 & 0xFF:02X} BOOT_CFG2=0x{(sbmr1 >> 8) & 0xFF:02X} '
          f'BOOT_CFG3=0x{(sbmr1 >> 16) & 0xFF:02X} BOOT_CFG4=0x{(sbmr1 >> 24) & 0xFF:02X}')
    print(f'SRC_SBMR2 = 0x{sbmr2:08X}  BMOD={bmod:02b} '
          f'({ {0: "boot from fuses", 1: "SERIAL DOWNLOADER", 2: "internal boot", 3: "reserved"}[bmod] })  '
          f'BT_FUSE_SEL={bt_fuse_sel}')
    print(f'SRC_SRSR  = 0x{srsr:08X}  (reset cause bits)')

    print('== step 4: FlexSPI NAND — configure like flashtool, then read')
    for addr, word in NAND_OPTION_WORDS:
        bl('fill-memory', hex(addr), 4, hex(word), quiet=True)
    cfg = bl('configure-memory', NAND_MEM_ID, '0x10000', check=False)
    if 'Success' not in cfg:
        print('!! configure-memory failed — NAND not answering with flashtool\'s geometry')
        return
    bl('get-property', '0x19', NAND_MEM_ID, check=False)
    for name, off, size in (('fcb_block0', 0x0, 0x1000),
                            ('image_copy_a', None, 0x1000),
                            ('image_copy_b', None, 0x1000)):
        if off is None:
            continue  # filled in below once block size is known
        path = os.path.join(OUT, f'nand_{name}.bin')
        r = bl('read-memory', hex(off), size, path, NAND_MEM_ID, check=False)
        if os.path.exists(path):
            d = open(path, 'rb').read()
            print(f'  {name}: {len(d)} B, first 16: {d[:16].hex(" ")}  tag@0x400: {d[0x400:0x404].hex(" ")}')
    print('Done.  Reload firmware with: venv-flash/bin/python scripts/flashtool.py -e sentai_runtime --ram')


if __name__ == '__main__':
    main()
