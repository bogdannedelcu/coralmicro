---
name: PXP needs PXP_Init() if used before camera.init()
description: PXP is held in SFTRST+CLKGATE on boot; BOARD_InitPxp() runs only in camera init path. Any other consumer must call PXP_Init() itself.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
PXP boots in **SFTRST + CLKGATE** state (`PXP_CTRL = 0xC0000000`).
The single de-init site is `BOARD_InitPxp()` in
`libs/camera/camera_support.c:761` — called only from
`CameraTask::HandleEnableRequest()` (i.e. the first
`sentai.camera.init(...)`).

If a different PXP consumer runs before camera.init (e.g.
`sentai.diag.aruco_bench` at fresh boot), `PXP_Start()` is a silent
no-op against a powered-down peripheral.  Symptom: the `kPXP_CompleteFlag`
busy-wait never finishes — board hangs (or hits whatever timeout the
caller has).

**Why:** PXP_Init() is what de-asserts SFTRST and CLKGATE.  Camera
authoritatively does it; everyone else must too.

**How to apply:** any new C++ code that uses PXP outside the camera
task should `PXP_Init(DEMO_PXP)` once before configuration.  The call
is idempotent — safe if camera already ran it.  See
`examples/sentai_runtime/aruco_bench.cc:aruco_bench_run` (build #1271)
for the canonical fix.

Diagnostic registers (memorise — easier than re-discovering):
- `PXP_CTRL = 0x40814000`  bit 31 = SFTRST, bit 30 = CLKGATE, bit 0 = ENABLE
- `PXP_STAT = 0x40814010`  bit 0 = IRQ0 (CompleteFlag), bit 3 = NEXT_IRQ
  bit 19 = BUFFER_VALID0 (informational)

Pre-fix capture (board #1270): CTRL=0xC0000000, STAT=0x0, iters=2 751 555
(full DWT timeout).
Post-fix (#1271): CTRL_before=0x0, STAT_after=0x80001, iters=3 682.
