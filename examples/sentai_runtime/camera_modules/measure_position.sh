#!/usr/bin/env bash
# W24-T22: measure both cameras with the production register set in the CURRENT physical
# position of the board (e.g. after rotating it 180 deg so cam0 / cam1 swap their place in
# front of the display): reset + init (the /cam files apply), re-localise (Gray codes), grid
# measurement in the common screen rect, comparison image.
# usage: bash measure_position.sh <tag>        -> out/measure_<tag>/ + out/compare_<tag>.png
set -u
HERE=$(cd "$(dirname "$0")" && pwd); S265=$HERE/../experiments/s265_camera_color_calibration
TAG=${1:?tag}; OUT=$HERE/out/measure_$TAG; mkdir -p "$OUT"
cd "$HERE"
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
r = Repl(); print('init ->', r.cmd('print(sentai.camera.init(1, 30))', 40).replace('\r', '').strip().split('\n')[-1]); time.sleep(2)
dm = r.cmd('print(sentai.rtos.dmesg(4000))', 20); print('applied:', sorted(set(re.findall(r'E:0A1[34]:\d+', dm))))
r.close()
PY
touch /tmp/mps_pause
echo "== locate"
(cd "$S265" && python3 locate.py --out "$OUT" 2>&1 | grep -v Warning | grep -E "^cam|common|residual|Error|Traceback")
REGION=$(python3 -c "import json; c=json.load(open('$OUT/locate.json'))['common_rect']; print(','.join(str(int(round(v))) for v in c))")
echo "== measure (grid, region $REGION)"
(cd "$S265" && python3 tune_colors.py --out "$OUT" --cams 0,1 --stages measure --region "$REGION" --avg 2 2>&1 | grep -v Warning | grep -E "measure:|abort|Traceback" | cut -c1-90)
(cd "$S265" && python3 show_pattern.py --quit >/dev/null 2>&1)
python3 compare_patches.py "$OUT" --out "out/compare_$TAG.png" --title "$TAG — production set, reference | cam0 | cam1"
