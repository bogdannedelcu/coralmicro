#!/bin/bash
# s145 — Mission migration template runner.
#
# Two modes:
#   --offline   (default)  Runs without Gazebo / SITL.  cf2 commands hit
#                          a non-listening UDP port; the template still
#                          executes all 6 phases (init returns 0, sends
#                          return 0 or -3 — both tolerated).  This mode
#                          validates the TEMPLATE STRUCTURE.
#   --live                 Brings up Gazebo + cf2 SITL via
#                          sim/scripts/launch_hybrid_cf2.sh, then runs
#                          the mission.  drone PHYSICALLY flies.
#                          Validates the END-TO-END FLOW.
#
# Exit 0 on PASS, 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
MODE="${1:---offline}"

SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"

MISSION_FILE="$SCRIPT_DIR/mission_template.py"
VERDICT="$SCRIPT_DIR/verdict.py"

WORKDIR="/tmp/s145_mission_template"
mkdir -p "$WORKDIR"

# ─── Stage mission file in SIM virtual FS ──────────────────────────
prep() {
    echo "[s145] staging mission_template.py → $FS_ROOT/"
    cp "$MISSION_FILE" "$FS_ROOT/"
    # Clean stale journal/summary so verdict sees this run only.
    rm -f "$FS_ROOT/mission_template_journal.txt" \
          "$FS_ROOT/mission_template_summary.json"
}

# ─── Live SITL bring-up (only for --live) ──────────────────────────
is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s145] cf2 SITL already up on UDP 19850"
        return 0
    fi
    echo "[s145] launching SITL stack"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" sentai_crazysim \
        > "$WORKDIR/sitl.log" 2>&1 &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s145] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s145] cf2 UDP 19850 ready"
}

teardown_sitl() {
    [ -x "$STOP_SH" ] && bash "$STOP_SH" > /dev/null 2>&1 || true
}

# ─── Mission invocation ────────────────────────────────────────────
run_mission() {
    echo "[s145] running mission_template via sentai_sim REPL"
    # Single-line import is the canonical pattern (per
    # [[sim-repl-test-recipe]]).  The mission's `finally` block writes
    # summary.json and closes the journal even on exception.
    #
    # Trailing newline + EOF causes sentai_sim REPL to exit cleanly.
    echo "import mission_template; r = mission_template.run(); print('FINAL:', r['status'])" \
        | timeout 90 "$SIM_BIN" > "$WORKDIR/repl.log" 2>&1
    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "[s145] sentai_sim rc=$rc — see $WORKDIR/repl.log"
        return $rc
    fi
}

verdict() {
    python3 "$VERDICT" "$FS_ROOT"
}

# ─── Main ──────────────────────────────────────────────────────────
case "$MODE" in
    --offline)
        echo "[s145] mode = OFFLINE (no Gazebo, structural validation only)"
        prep
        run_mission && verdict
        ;;
    --live)
        echo "[s145] mode = LIVE (Gazebo + cf2 SITL)"
        prep
        ensure_sitl_up || exit 1
        # Live mode: cf2 SITL listens, drone physically flies, summary
        # captures actual go_to executions.
        run_mission
        teardown_sitl
        verdict
        ;;
    *)
        echo "Usage: $0 [--offline|--live]"
        exit 2
        ;;
esac
