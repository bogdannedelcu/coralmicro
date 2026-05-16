# t_template_offline.py — SIM-only smoke for mission_template (Task #40).
#
# Runs the canonical template against an offline cf2 (no Gazebo, no SITL).
# Validates:
#   1. The template file imports cleanly inside sentai_sim REPL.
#   2. run() executes all 6 phases without unhandled exception.
#   3. summary.json + journal.txt are written to sentai_fs_root.
#   4. status == "DONE" (UDP send succeeds even with no peer; failures
#      come back as send rc=-3 but the template ignores those).
#
# Run from the SIM REPL: `import t_template_offline`.

import sentai
import mission_template

print("---- t_template_offline ----")

# 1. Execute the mission against offline cf2.
result = mission_template.run()

# 2. Verify shape of returned summary.
assert isinstance(result, dict), "run() must return dict"
print("status:", result["status"])
print("phases:", result["phases_done"])
print("phase_count:", result["phase_count"])
print("waypoints:", result["waypoints_visited"])
if result.get("errors"):
    print("errors:", result["errors"])

assert result["status"] == "DONE", "expected status=DONE, got %s" % result["status"]
assert result["phase_count"] == 6, \
    "expected 6 phases, got %d (%s)" % (result["phase_count"], result["phases_done"])
assert result["waypoints_visited"] == 2, \
    "expected 2 waypoints, got %d" % result["waypoints_visited"]

# 3. Verify summary.json was written to the SIM virtual FS.
assert sentai.fs.exists(mission_template.SUMMARY_NAME), \
    "summary file not written: %s" % mission_template.SUMMARY_NAME
size = sentai.fs.size(mission_template.SUMMARY_NAME)
print("summary file size:", size)
assert size > 50, "summary file suspiciously small"

# 4. Verify journal was written + closed.
assert sentai.fs.exists(mission_template.JOURNAL_NAME), \
    "journal file not written: %s" % mission_template.JOURNAL_NAME
j_size = sentai.fs.size(mission_template.JOURNAL_NAME)
print("journal file size:", j_size)
assert j_size > 100, "journal file suspiciously small"

# Sanity: journal contains key events.
journal_text = sentai.fs.read_str(mission_template.JOURNAL_NAME)
for ev in ("crazy_init", "crazy_arm", "crazy_takeoff",
           "waypoint_start", "waypoint_done", "crazy_land", "crazy_disarm"):
    assert ev in journal_text, "missing journal event: %s" % ev
print("journal contains all expected events")

print("---- t_template_offline PASS ----")
