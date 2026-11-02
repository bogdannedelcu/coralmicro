#!/usr/bin/env bash
# SXGA time-code runs on build >= 1760 (peek_col(x, byte)); locate from iter4_sxga15 (same geometry).
set -u
HERE=$(cd "$(dirname "$0")" && pwd); cd "$HERE"
init_board() {  # $1 fps, $2 packed(0/1)
python3 - "$1" "$2" <<'PY'
import sys, time, serial, glob
sys.path.insert(0, '../s265_camera_color_calibration')
from board_repl import Repl
fps, packed = int(sys.argv[1]), sys.argv[2] == '1'
r = Repl(); print('version', r.value('sentai.version()')); r.ser.write(b'sentai.sys.reset()\r'); time.sleep(0.5); r.close()
t0 = time.time()
while time.time() - t0 < 40:
    time.sleep(1)
    try:
        s = serial.Serial(glob.glob('/dev/ttyACM*')[0], 115200, timeout=0.5); s.close(); break
    except Exception:
        pass
time.sleep(3)
r = Repl()
print('set_hw ->', r.value('sentai.camera.set_hw(1280, 960)'))
if packed:
    print('packed420(1) ->', r.value('sentai.camera.packed420(1)'))
print('init(1, %d) ->' % fps, r.cmd('print(sentai.camera.init(1, %d))' % fps, 30).split('\n')[-1].strip())
time.sleep(2)
print('resolution', r.value('sentai.camera.resolution()'), 'peek rows', r.value('len(sentai.camera.peek_col(40, 1)[2])'))
r.close()
PY
}
run_tc() {  # $1 out, $2 exp, $3 gain, $4 byte
mkdir -p "$1"; python3 timecode_display.py --quit 2>/dev/null; rm -f "$1/display_log.csv"
nohup python3 timecode_display.py --log "$1/display_log.csv" > "$1/display.out" 2>&1 &
sleep 4
python3 run_timecode.py --out "$1" --seconds 30 --exp-lines "$2" --gain "$3" --byte "$4" --locate iter4_sxga15/locate.json 2>&1 | grep -vE '^$|RuntimeWarning|_methods|fromnumeric|bands \(rows'
python3 timecode_display.py --quit 2>/dev/null
}
# usage: bash run_sxga.sh [out_dir_sxga15] [out_dir_sxga30p]   (defaults = the build-1760 iterations)
OUT15=${1:-iter4_sxga15}; OUT30=${2:-iter5_sxga30}
echo "===== SXGA 15 legacy ====="; init_board 15 0; run_tc "$OUT15" 50 160 0
echo "===== SXGA 30 packed (byte 1 = Y1) ====="; init_board 30 1; run_tc "$OUT30" 100 160 1
