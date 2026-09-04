---
name: fnirsi-power-logger-already-running
description: "A continuous FNIRSI FNB48S power log of the SentAI board already exists at /tmp/power_log.jsonl — do not start a second logger, slice that one."
metadata: 
  node_type: memory
  type: reference
  originSessionId: d5a3f91c-f734-4950-82fe-b796eef65638
  modified: 2026-09-04T08:45:46.818Z
---

The SentAI board is powered **through** an FNIRSI FNB48S USB meter
(`lsusb` → `2e3c:0049`), and a logger has been running continuously since
**2026-08-18**:

```
python3 tools/fnirsi_power.py     # cwd /home/bogdan/work/ch569w
```

- Appends every sample at 100 SPS to **`/tmp/power_log.jsonl`** (~11 GB by
  2026-09-04): `{"t": unix_ts, "v", "i", "p", "temp"}`.
- Serves a live dashboard on **http://localhost:8081/** with `/stats`,
  `/samples?since=T`, `POST /reset` and **`POST /mark?label=...`**.

**Do not start a second logger and do not kill this process** — it holds the
meter exclusively, so any other client gets `OSError: open failed`. The device
shows up bound to `usbfs` (no hidraw node), which is that process holding it,
not a misconfiguration; rebinding to `usbhid` does not take while it runs.

**How to use it for a board experiment** (pattern from EXP-s246 iter3,
`run_with_power.py`): `POST /mark?label=start`, run the experiment, `POST
/mark?label=end`, then slice `[t0, t1]` out of the JSONL by seeking near the
end of the file and scanning backwards in blocks — never read the 11 GB whole.

**What it settled once:** during a 47.9 s sustained 45 fps dual-camera sweep the
rail held at **V min 4.988 V** (mean 5.039), P mean 1.834 W / max 1.915 W. So
the mid-run board resets seen that session were a **software fault, not
brownout** — a question that is otherwise indistinguishable from the REPL.
Board idle is ≈0.79 W; this sweep ran ≈1.83 W.

Separate, paper-grade tooling lives in `/home/bogdan/work/appliedsci/`
(`energy_capture.py`, `fnb_log.py`) and is covered by the `energy-measure`
skill — but both open the meter by VID/PID and will fail while the ch569w
logger holds it.

Related: [[frame-count-half-rate-and-alternation-tag-bug]].
