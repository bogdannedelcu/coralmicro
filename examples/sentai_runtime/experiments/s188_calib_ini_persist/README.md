# s188 — INI persistence round-trip + forward-compat

**WBS:** `OP-S10-W21-T2`.  Focused tests on the INI persistance layer
(`/system/calib.ini`) that replaces the legacy JSON `cam_calib.json`.
s157 / s186 already exercise commit_R + save + clear + load (T6 in
each); s188 adds the cases unique to the format swap:

| # | Scenario                                            | Pass criterion                        |
|---|-----------------------------------------------------|----------------------------------------|
| T1 | On-disk format is INI (newline-separated key=val)  | Lines match expected regex             |
| T2 | Schema mismatch (`schema=99`) rejected             | load() returns 0, R unchanged          |
| T3 | Forward-compat: unknown keys ignored                | Synthetic INI with extra `kp_x=0.39`, `mystery=42` still parses R + offset correctly |
| T4 | Empty / truncated file rejected                     | load() returns 0                       |
| T5 | Comment lines (`#`, `;`) tolerated                 | INI with comment lines parses OK       |

Pure synthetic; uses MP `sentai.fs.write` to forge files in the
sentai_fs_root cwd, then calls `sentai.calib.load()` to exercise the
parser.

## How to run

```bash
bash examples/sentai_runtime/experiments/s188_calib_ini_persist/run.sh
```
