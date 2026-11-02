#!/usr/bin/env bash
# W24-T22: measure the colour error of both cameras on the Kamvas colour grid for three
# production variants of the per-module register files, each applied by camera.init()
# from /cam/camN.regs after a board reset:
#   auto   — production default: no AEC/AGC freeze, no manual AWB (make_regs.py)
#   manual — tuned manual AWB gains kept (make_regs.py --awb manual)
#   frozen — full tuning-session set incl. AEC/AGC freeze (--awb manual --keep-exposure), diagnostic
# usage: bash measure_variants.sh <out_tag>     -> out/measure_<tag>_<variant>/
set -u
HERE=$(cd "$(dirname "$0")" && pwd); S265=$HERE/../experiments/s265_camera_color_calibration
TAG=${1:-v2}
reset_init() {
python3 - <<'PY' 2>&1 | grep -v Warning
import sys, time, glob, serial, re
sys.path.insert(0, '../experiments/s265_camera_color_calibration')
from board_repl import Repl
r = Repl(); r.ser.write(b'sentai.sys.reset()\r'); time.sleep(0.5); r.close()
t0 = time.time()
while time.time() - t0 < 40:
    time.sleep(1)
    try:
        s = serial.Serial(sorted(glob.glob('/dev/ttyACM*'))[0], 115200, timeout=0.5); s.close(); break
    except Exception:
        pass
time.sleep(3)
r = Repl(); print('  init ->', r.cmd('print(sentai.camera.init(1, 30))', 40).replace('\r', '').strip().split('\n')[-1]); time.sleep(2)
dm = r.cmd('print(sentai.rtos.dmesg(4000))', 20); print('  applied:', sorted(set(re.findall(r'E:0A1[34]:\d+', dm))), ' 0x3503 cam0/cam1:', r.reg(0, 0x3503), r.reg(1, 0x3503), ' 0x3406:', r.reg(0, 0x3406), r.reg(1, 0x3406))
r.close()
PY
}
cd "$HERE"
for V in auto manual frozen; do
  case $V in
    auto)   FLAGS="";;
    manual) FLAGS="--awb manual";;
    frozen) FLAGS="--awb manual --keep-exposure";;
  esac
  echo "===== variant $V ($FLAGS)"
  python3 make_regs.py $FLAGS --out out/regs_$V --upload 2>&1 | grep -v Warning | grep uploaded
  reset_init
  OUT=out/measure_${TAG}_$V; mkdir -p "$OUT"; cp "$S265/iter3_kamvas/locate.json" "$OUT/"
  (cd "$S265" && python3 tune_colors.py --out "$HERE/$OUT" --cams 0,1 --stages measure --region 860,46,794,923 --avg 2 2>&1 | grep -v Warning | grep -E "measure:|abort|Traceback" | cut -c1-70)
  (cd "$S265" && python3 show_pattern.py --quit >/dev/null 2>&1)
done
echo "===== restore production default (auto) on the card"
python3 make_regs.py --upload 2>&1 | grep uploaded
python3 - "$TAG" <<'PY'
import json, numpy as np, sys
tag = sys.argv[1]
print('%-8s %-5s %-9s %-9s %-11s' % ('variant', 'cam', 'dE76', 'dE00', 'greys dE00'))
for v in ('auto', 'manual', 'frozen'):
    for cam in (0, 1):
        d = json.load(open('out/measure_%s_%s/measure_patches_cam%d.json' % (tag, v, cam)))
        g = [m['dE00'] for k, m in d.items() if 'neutral' in k.lower() or k.lower() in ('white', 'black')]
        print('%-8s cam%d  %-9.1f %-9.1f %-11.1f' % (v, cam, np.mean([m['dE76'] for m in d.values()]), np.mean([m['dE00'] for m in d.values()]), np.mean(g)))
PY
