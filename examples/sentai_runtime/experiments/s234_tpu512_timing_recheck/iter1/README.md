# s234 iter1 - unchanged `_t_timing.py` re-run

**Hypothesis**: The original timing driver reproduces the previously reported
pure TPU and pipeline timing split closely enough to identify whether Table 9
was an artifact of method or a stable measurement.

**Change vs prior iter**: baseline.

**Date**: 2026-06-24

## Files Captured

- `lsusb_before.txt` - USB enumeration before the run.
- `git_status.txt` - local workspace state.
- `board_probe.log` - REPL probe with `sentai.version()` and basic TPU config.
- `upload_t_timing.log` - upload transcript for unchanged `_t_timing.py`.
- `fs_probe.log` - REPL probe showing the board user FS is unavailable.
- `blocked_summary.json` - machine-readable blocked state.

## Result

Blocked before the experiment body ran.

The unchanged `_t_timing.py` upload failed because
`sentai.fs.append("/lib/diag/_t_timing.py", ...)` returned `False`.
Follow-up probing showed that the user FS root itself is unavailable:
`sentai.fs.ls("/")` raises `OSError: dir not found`, while `mkdir`,
`write`, and `append` all return `False`.
