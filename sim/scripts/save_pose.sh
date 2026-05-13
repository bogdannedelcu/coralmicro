#!/bin/bash
# save_pose.sh — record cf2 pose from gz topic to <out_dir>/pose.csv
# Runs INSIDE distrobox crazysim-garden.
#
# Usage: save_pose.sh <out_dir> <world_name> [seconds]
set -e

OUT="${1:?usage: save_pose.sh <out_dir> <world_name> [seconds]}"
WORLD="${2:?usage: save_pose.sh <out_dir> <world_name> [seconds]}"
SECS="${3:-60}"
mkdir -p "$OUT"

CSV="$OUT/pose.csv"
echo "t_sec,x,y,z,qx,qy,qz,qw" > "$CSV"
echo "[pose] recording to $CSV for ${SECS}s" >&2

# gz topic -e prints text-protobuf for Pose_V; we want the crazyflie_0 entry.
# Parse with awk — extract sec, name, position {x,y,z}, orientation {x,y,z,w}.
exec timeout "$SECS" gz topic -e -t "/world/$WORLD/pose/info" | awk -v csv="$CSV" '
function emit() {
  if (saw_cf && t_sec != "") {
    printf("%s,%s,%s,%s,%s,%s,%s,%s\n", t_sec, x, y, z, qx, qy, qz, qw) >> csv;
    fflush(csv);
  }
  saw_cf = 0; t_sec=""; x=""; y=""; z=""; qx=""; qy=""; qz=""; qw="";
}
/^header/ { in_h=1 }
/^pose/   { in_pose=1; in_h=0 }
in_h && /^[ \t]*sec:/ { gsub(/[^0-9]/,"",$0); t_sec = $0 }
in_pose && /name: "crazyflie_0"/ { saw_cf=1 }
saw_cf && in_pose && /position/ { in_pos=1; in_ori=0 }
saw_cf && in_pose && /orientation/ { in_ori=1; in_pos=0 }
in_pos && /^[ \t]*x:/ { gsub(/[^0-9.e+-]/,"",$2); x=$2 }
in_pos && /^[ \t]*y:/ { gsub(/[^0-9.e+-]/,"",$2); y=$2 }
in_pos && /^[ \t]*z:/ { gsub(/[^0-9.e+-]/,"",$2); z=$2 }
in_ori && /^[ \t]*x:/ { gsub(/[^0-9.e+-]/,"",$2); qx=$2 }
in_ori && /^[ \t]*y:/ { gsub(/[^0-9.e+-]/,"",$2); qy=$2 }
in_ori && /^[ \t]*z:/ { gsub(/[^0-9.e+-]/,"",$2); qz=$2 }
in_ori && /^[ \t]*w:/ { gsub(/[^0-9.e+-]/,"",$2); qw=$2; emit(); in_ori=0; in_pos=0 }
'
