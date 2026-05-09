# main.py — MISSION code only.  Auto-runs after firmware boot.
#
# IMPORTANT: the radio bridge (sentai.crazy.*) is started by the
# FIRMWARE before this file runs, in micropython_repl_task — see
# the auto-init block in micropython_task.c.  Do NOT add
# sentai.crazy.init() here: it would be redundant (idempotent
# no-op), and more critically the firmware-side init guarantees
# radio recovery even if THIS file is missing, truncated, or
# raises an exception.  Per agent.md §18 + embeded.md §M, radio
# is the only remote-recovery path for a drone-deployed board.
#
# What this file SHOULD contain: mission features (camera, flow,
# experiment-specific setup).  If this file is corrupted or
# raises, the board still boots into a radio-reachable REPL and
# the operator can fix it over CRTP $-exec.
#
# To disable mission auto-init temporarily over radio:
#   $sentai.fs.remove('/main.py')
#   $sentai.sys.reset()
# After fix, durable upload via REPL chunked uploader + sync():
#   python3 diag/_host_upload_repl.py --file ...
#   ($sync is auto-called by the uploader after the last chunk)

import sentai

sentai.camera.init()
sentai.flow.enable()
sentai.flow.start(0)
