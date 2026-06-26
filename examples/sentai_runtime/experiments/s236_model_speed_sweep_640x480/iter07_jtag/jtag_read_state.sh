#!/usr/bin/env bash
# s236 iter07 - JTAG read of M7 TPU-driver state after a failed c2f_thick invoke.
# Run in YOUR terminal (sudo if J-Link needs it). Writes results to jtag_out.txt.
#
#   bash jtag_read_state.sh
#   (or: sudo bash jtag_read_state.sh   if J-Link permission errors)
#
# Step 1 triggers the c2f_thick failure over the REPL (leaves the globals set).
# Step 2 halts the M7 via J-Link and dumps the key symbols (build 1548 ELF):
#   0x20026de8 g_sentai_tpu_invoke_fail_code  (expect 0x0B63)
#   0x20026db8 g_sentai_tpu_by_output         (output bytes read before stall)
#   0x2002f12c g_sentai_tpu_take_failed       (bulk-IN timeout count)
#   0x81d45040 s_event_buf[16]                (16-byte TPU completion event)
set -u
cd "$(dirname "$0")/../.." || exit 1   # -> experiments/s236.../
EXP="$(pwd)/s236_model_speed_sweep_640x480" 2>/dev/null || EXP="$(pwd)"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/jtag_out.txt"
: > "$OUT"

echo "=== STEP 1: trigger c2f_thick failure over REPL ===" | tee -a "$OUT"
python3 - "$HERE" <<'PY' 2>&1 | tee -a "$OUT"
import sys, time, subprocess
sys.path.insert(0, "/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s236_model_speed_sweep_640x480")
import board_serial as B
# fresh reset for a clean single-pass-of-failure state
try:
    s=B.open_repl(); B.cmd(s,"import sentai",4); s.write(b"sentai.sys.reset()\r\n"); time.sleep(1); s.close()
except Exception: pass
for _ in range(30):
    if b"1fc9:c0a1" in subprocess.run(["lsusb"],capture_output=True).stdout: break
    time.sleep(1)
time.sleep(6)
ser=None
for a in range(12):
    try: ser=B.open_repl(); break
    except Exception: time.sleep(2)
def p(l,t=18):
    o=B.cmd(ser,l,t); return " | ".join([x for x in o.split("\r\n") if x and not x.startswith(">>>") and x.strip()!=l.strip()][-2:])
time.sleep(1.0); B.cmd(ser,"",2)
print("imp :", p("import sentai"))
print("ver :", p("print(sentai.version())"))
print("load:", p("print('L', sentai.tpu.load('/c2f_thick.tflite'))"))
p("sentai.camera.set_resolution(640,480)",4); p("sentai.camera.init(1)",15); time.sleep(1); p("sentai.camera.to_tensor()",10)
print("INV :", p("print('INV', sentai.tpu.invoke())",22))
try: ser.close()
except Exception: pass
print(">>> board left running in post-fail state (DO NOT reset)")
PY

echo "" | tee -a "$OUT"
echo "=== STEP 2: J-Link halt + read globals ===" | tee -a "$OUT"
JL="$HERE/jl_read.jlink"
cat > "$JL" <<'EOF'
halt
regs
mem32 0x20026de8 1
mem32 0x20026db8 1
mem32 0x2002f12c 1
mem8 0x81d45040 16
go
qc
EOF
JLinkExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -autoconnect 1 -NoGui 1 \
  -CommandFile "$JL" 2>&1 | tee -a "$OUT"

echo "" | tee -a "$OUT"
echo "=== DONE. Results in: $OUT ===" | tee -a "$OUT"
echo "Decode: invoke_fail_code @20026de8 should be 0x0B63 (GetOutputs)."
echo "        by_output @20026db8 = output bytes read before stall."
echo "        take_failed @2002f12c = bulk-IN timeouts."
echo "        s_event_buf @81d45040 = the 16-byte TPU completion event."
