---
name: Radio CRTP MTU 30 B = inline $exec only, no file transfer
description: Pre-load files via USB before drone deployment. Once on radio, only short $exec + alias trick + telemetry. Don't try to ship files over CRTP.
type: feedback
originSessionId: c7a210f2-a5f7-4a53-86b4-b15f5f9be074
---
When the SentAI board is on drone battery (USB unplugged), the radio
path is for **short inline `$exec` commands and telemetry queries
ONLY**.  Do NOT attempt file transfer over radio.

**Why:** CRTP payload max = 30 B per packet.  After `$sentai.fs.append('/m.py', b'')` overhead, ~5-6 useful bytes of payload remain.  A
1 KB script needs ~300+ round-trips.  At 50 ms RTT that's ~15 s
minimum, eating radio bandwidth needed for control + telemetry.
Any dropped packet mid-chunk silently truncates the file and
`sentai.fs.sync()` will commit the truncated state.

**How to apply (deployment workflow):**
1. **Before flight**: pre-load all driver/config files via USB
   (`diag/_host_upload_repl.py` + explicit `sentai.fs.sync()`).
2. `/main.py` on board = mission-code auto-init point (camera, flow,
   experiment-specific setup).  Recovery infrastructure (crazy bridge,
   USB CDC, WDOG) is in firmware, not main.py — survives main.py
   corruption.
3. **In flight (radio only)**: only short inline `$exec`, telemetry
   queries (`sentai.crazy.battery()` etc.), and one-shot status
   commands.

**Inline-exec mechanics that work** (validated 2026-05-09 motion test
on drone battery — `/tmp/test_flow_motion_radio.py`):

```
$1+1                                # 2 B util — sanity
$sentai.flow.pub_stats()            # 24 B → multi-fragment auto-reasm
$sentai.camera.init()               # 21 B
$sentai.flow.start(0)               # 21 B
$sentai.io.led_on()                 # 19 B
$sentai.io.led_off()                # 20 B
$sentai.crazy.battery()             # 23 B → telemetry float
```

**Alias trick** for >29 B calls (CRTP MTU forces this):

```
$r=sentai.flow.read       (19 B)   then  $r()              (5 B)
$f=sentai.crazy.send_flow (24 B)   then  $f(dx,dy,dt,std)  (~22 B)
$l=sentai.io.led_on       (19 B)   then  $l()              (5 B)
$o=sentai.io.led_off      (20 B)   then  $o()              (5 B)
```

Aliases live in MP REPL globals → persist across radio exec calls
(same MP VM context).  Lost only on board reset.

**Reply fragmentation is transparent** (board → host).  `link_send`
walks the reply buffer, prepends MF=1 to all but last fragment,
MF=0 to last.  Host reassembles:

```python
fragments = []
def cb(pkt): fragments.append(bytes(pkt.data))
cf.add_port_callback(0x0E, cb)
# ... send packet, wait, then:
joined = b''.join(f[1:] for f in fragments).decode('utf-8')
```

**Realistic polling rate** at 50 ms RTT baseline:
- 1-fragment reply: ~30 ms RTT — 30 Hz max
- 6-fragment reply (sentai.flow.read ~150 B): ~120 ms — ~6 Hz max
- 9-fragment reply (sentai.diag.dmesg slice): ~180 ms — ~5 Hz

Don't try to drive a fast control loop over radio.  It's for
SUPERVISION, not real-time.  For ~30 Hz flow injection, use
on-board `sentai.crazy.send_flow()` from a publisher task that
sources data from `sentai.flow.read()` directly — no host involved.

**Theoretical out-of-MTU path** (NOT implemented, available if needed):
host-side outbound MF fragmentation.  Send `$sentai.fs.append('/m.py',
b'<chunk>')` as multiple CH=0 packets with MF=1 on all-but-last.
Board reassembles before dispatch.  Useful for occasional config
updates over radio when re-flashing isn't an option.  Cost: ~30
packets × ~50 ms = ~1.5 s per 1 KB.  See agent.md §18 best-practice
section for the recipe sketch.

**Also see:** `project_crazyflie_radio_bridge.md` for bridge architecture
+ build state, `agent.md §18` for operational reference.
