#!/usr/bin/env python3
"""s111 — Read PXP debug capture from aruco_bench via REPL.

Why: PXP_Start() never sets kPXP_CompleteFlag in s_kernel_pxp_y8_dummy.
We added register capture (PXP_CTRL/PXP_STAT before+after) and a
sanity-check that aborts if PXP_CTRL.ENABLE is already set on entry
(returns sentinel 0xDEAD).

Output interpretation:
  pxp_ctrl_before & 0x1   -> PXP already running on entry (busy from cam?)
  pxp_ctrl_after  & 0x1   -> ENABLE still set => PXP started but never finished
  pxp_stat_after  & 0x1   -> IRQ0 (CompleteFlag) set?
  pxp_stat_after  & 0x8   -> NEXT_IRQ — pending config error
  pxp_iters               -> busy-wait count (huge = full 125 ms timeout)

Pass criteria for the read itself: all 4 keys print without REPL chunking.
"""
import serial, time, sys

PORT = '/dev/ttyACM0'

s = serial.Serial(PORT, 115200, timeout=5)

def drain():
    time.sleep(0.4)
    return s.read(s.in_waiting or 1)

def send(cmd, wait=1.0):
    s.write(cmd.encode() + b'\r\n')
    time.sleep(wait)
    return s.read(s.in_waiting or 1).decode(errors='replace')

# Clean REPL state
for _ in range(3):
    s.write(b'\x03')
    time.sleep(0.3)
drain()

print("[s111] running sentai.diag.aruco_bench() ...")
out = send('r=sentai.diag.aruco_bench()', wait=4.5)
sys.stdout.write(out)

keys_dec = ['thresh_us', 'thresh_bradley_us', 'thresh_separable_us',
            'thresh_pxp_us', 'thresh_pxp_simd_us',
            'thresh_pxp_simd_staged_us',
            'pxp_rectify_us', 'cmsis_dsp_demo_us',
            'pxp_iters', 'edge_us', 'scan_us']
keys_hex = ['pxp_ctrl_before', 'pxp_stat_before',
            'pxp_ctrl_after', 'pxp_stat_after']

print("\n=== DECIMAL ===")
for k in keys_dec:
    out = send(f'print("{k}=",r.get("{k}","?"))', wait=0.8)
    sys.stdout.write(out)
    sys.stdout.flush()

print("\n=== HEX ===")
for k in keys_hex:
    out = send(f'print("{k}=",hex(r.get("{k}",0)))', wait=0.8)
    sys.stdout.write(out)
    sys.stdout.flush()

# Final drain
time.sleep(1.0)
print("\n=== DRAIN ===")
print(s.read(s.in_waiting or 1).decode(errors='replace'))
