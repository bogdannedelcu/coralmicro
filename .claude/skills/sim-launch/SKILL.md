---
name: sim-launch
description: Kill any running Gazebo + launch a SIM world with GUI + camera bridge + cf2 SITL. Codifies the anti-orphan and GUI-required rules before any SIM experiment. Anti-cheat-aware.
---

# /sim-launch

## Pre-flight rules (load-bearing)

- `[[kill-all-gz-before-launch]]` — orphan gz topics from a prior run silently corrupt the new session.
- `[[gazebo-gui-required]]` — headless Gazebo runs are NOT valid evidence (operator-reaffirmed 2026-05-14).
- `[[sentai-sim-air-gapped-from-truth]]` — verify the cheat plugin is NOT in the world file BEFORE launching.

## Steps

1. **Kill all gz + simulator processes:**
   ```bash
   pkill -9 -f 'gz-sim|gz-tools|sentai_sim|gt_recorder' || true
   sleep 1
   ```

2. **Audit anti-cheat for the world file:**
   ```bash
   bash /home/bogdan/work/coralmicro/sim/scripts/audit_anti_cheat.sh
   ```
   Must pass (exit 0) before launching.

3. **Launch Gazebo with GUI:**
   ```bash
   gz sim -g --gui-config /home/bogdan/work/coralmicro/sim/gazebo/sentai_gui.config <world_path> &
   ```

4. **Launch camera bridge** (gz → UDS for sentai_sim).  Bridge hard-rejects non-camera topics at startup per anti-cheat:
   ```bash
   python3 /home/bogdan/work/coralmicro/sim/scripts/gz_to_uds_bridge.py &
   ```

5. **Launch cf2 SITL** (if mission needs it) — check world SDF for the cf2 model name.

6. **Launch GT recorder** (host-side ONLY, never injected into SIM, per anti-cheat):
   ```bash
   export GT_RECORDER_OUT=./gt.jsonl
   export GT_RECORDER_WORLD=<world>
   export GT_RECORDER_MODEL=<model>
   python3 /home/bogdan/work/coralmicro/sim/scripts/gt_recorder.py &
   ```

7. **Launch sentai_sim:**
   ```bash
   /home/bogdan/work/coralmicro/build-sim/sentai_sim
   ```

## Verify before declaring "launched"

- `gz topic -l` lists ONLY expected camera topics; no `/world/.../dynamic_pose/info` subscribed by sentai_sim.
- `lsof -p $(pgrep sentai_sim) | grep sentai_cam.sock` shows the UDS connection.
- Gazebo GUI window visible.

## REPL gotchas on SIM (line-buffered embed REPL)

The SIM REPL is **line-buffered** and runs the MicroPython embed port — it has known limitations that break recipes copied from ARM REPL workflows:

- **No multi-line `def` / `class` / `for:` blocks** — the REPL processes ONE line at a time and never enters multi-line mode.  `def foo():\n    return 1` fails on line 2.
- **No paste-mode** (`Ctrl-E`) — not implemented in embed port.
- **No `open()`** — SIM `sentai.fs` is POSIX-file-backed but stdlib `open()` is not exposed.
- **Multi-line expressions inside `[]` `()` `{}`** also fail — keep expressions on one physical line.

**Required pattern** to run anything non-trivial: drop a `.py` file into `build-sim/sentai_fs_root/` (NO leading underscore), then `import` it from REPL:

```bash
cp my_test.py /home/bogdan/work/coralmicro/build-sim/sentai_fs_root/my_test.py
# In REPL:
>>> import my_test
>>> my_test.main()
```

If you try to paste a multi-line `def` directly into the REPL, the test fails on the second line with `SyntaxError: invalid syntax`.

Pre-flight check before launching a test: if the test is more than 3 lines of Python and contains any `def`/`class`/`for:`/`while:` block, refactor to an importable file first.

## Cleanup

When done:
```bash
pkill -9 -f 'gz-sim|gz-tools|sentai_sim|gt_recorder'
```

## Reject patterns

- Skipping `pkill` — orphan topics will silently break the new session.
- Launching headless (`gz sim -s` without `-g`) — evidence does not count.
- Skipping the anti-cheat audit even for a "quick test".
- Letting the GT recorder write into a path that sentai_sim reads.

## See also

- `sim/ANTI_CHEAT.md`
- `[[gt-recorder-tool]]` (auto-memory)
- `Sim.md` §10x (SIM journal helper)
