#!/usr/bin/env bash
# EXP-s266: validate one camera mode end to end.
# usage: bash run_mode.sh <iter_dir> <fps> <exp_lines> <gain_1/16> [sxga] [packed]
set -u
HERE=$(cd "$(dirname "$0")" && pwd); S265=$HERE/../s265_camera_color_calibration
OUT=$1; FPS=$2; EXP=$3; GAIN=$4; SXGA=${5:-}; PACKED=${6:-}
mkdir -p "$OUT"; cd "$HERE"
python3 timecode_display.py --quit 2>/dev/null; python3 "$S265/show_pattern.py" --quit 2>/dev/null
python3 - "$FPS" "$SXGA" "$PACKED" <<'PY'
import sys, time, serial, glob
sys.path.insert(0, '../s265_camera_color_calibration')
from board_repl import Repl
fps, sxga, packed = int(sys.argv[1]), sys.argv[2] == 'sxga', sys.argv[3] == 'packed'
r = Repl(); r.ser.write(b'sentai.sys.reset()\r'); time.sleep(0.5); r.close()
t0 = time.time()
while time.time() - t0 < 40:
    time.sleep(1)
    try:
        s = serial.Serial(glob.glob('/dev/ttyACM*')[0], 115200, timeout=0.5); s.close(); break
    except Exception:
        pass
time.sleep(3)
r = Repl()
if sxga:
    print('set_hw(1280, 960) ->', r.value('sentai.camera.set_hw(1280, 960)'))
if packed:
    print('packed420(1) ->', r.value('sentai.camera.packed420(1)'))
print('init(1, %d) ->' % fps, r.cmd('print(sentai.camera.init(1, %d))' % fps, 30).split('\n')[-1].strip())
time.sleep(2)
print('period_us', r.value('sentai.camera.stats()["period_us"]'), 'resolution', r.value('sentai.camera.resolution()'), 'ratio_unit', r.value('sentai.camera.ratio_unit()'))
r.close()
PY
echo "== locate ($OUT) =="
python3 "$S265/locate.py" --out "$OUT" 2>&1 | grep -E '^cam|common'
python3 "$S265/show_pattern.py" --quit 2>/dev/null; sleep 0.5
rm -f "$OUT/display_log.csv"; nohup python3 timecode_display.py --log "$OUT/display_log.csv" > "$OUT/display.out" 2>&1 &
sleep 4
echo "== timecode ($OUT, exp $EXP lines, gain $GAIN/16) =="
python3 run_timecode.py --out "$OUT" --seconds 30 --exp-lines "$EXP" --gain "$GAIN" --locate "$OUT/locate.json" 2>&1 | grep -vE '^$|RuntimeWarning|_methods|fromnumeric|bands'
python3 timecode_display.py --quit 2>/dev/null
