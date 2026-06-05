---
name: feedback-emu-pixel-format-caveat
description: "HARD RULE — emulator scaffolding using small Y8 frames must not be claimed as production-format-equivalent. Production = XRGB8888 raw from CSI, PrepTask converts to Y8 80x60 before FLOW. Don't conflate."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8b2903f8-f695-46ef-9f13-edba1cccdfeb
---

Operator note 2026-06-02 (during B8.7c cat-flow review):

> "sa ai grija la pixel format da?"

Reminder: production SentAI camera pipeline carries frames in two
different formats at different stages, and the emulator gates should
not claim to model the wrong one:

| Stage                       | Format                                | Where                                                          |
| --------------------------- | ------------------------------------- | -------------------------------------------------------------- |
| OV5640 / MIPI CSI delivery  | **XRGB8888** raw (4 bytes/pixel)      | `libs/camera/camera_support.c` ISR + `libs/camera/camera.cc`   |
| PrepTask camera read        | XRGB8888 input → Y8 output            | `examples/sentai_runtime/sentai_prep.cc` + PXP                  |
| FLOW_GRAY slot              | **Y8 80x60** single byte / pixel      | `examples/sentai_runtime/flow_task.cc`                          |
| TPU input                   | INT8 quantised tensor                 | `examples/sentai_runtime/detection_task.cc`                    |

Errata RT1176 ERR051248: the CSI is hard-wired to deliver 24-bit pixels
as XRGB8888 in DRAM (no native RGB565 mode), so we cannot pretend the
camera ever delivers anything else.  See
`project_csi_rgb565_silicon_dead_end` memory entry.

How to apply for emulator (or any other test scaffolding):

- Do NOT call an 8-bit grayscale 32x32 input "production-equivalent flow
  input".  The closest production analogue is the Y8 80x60 slot, and
  that lives behind PrepTask + PXP + RGB→Y8 conversion, none of which
  this emu spike models.
- Tests that use Y8 small frames should state explicitly that they skip
  the camera/PrepTask conversion stage.  Header comments and B8 docs
  should call this out per gate.
- A future gate that actually exercises PrepTask conversion must inject
  XRGB8888 bytes, not Y8.
- The same reminder extends to other formats: 2D point lists for
  markers/ArUco have specific layouts in production, and a stand-in
  must not pretend to match by reusing the production struct name.

Cross-refs: [[feedback_emu_must_not_reuse_production_task_names]] (same
spirit: scaffolding stays visibly distinct from production),
[[project_csi_rgb565_silicon_dead_end]], [[project_op_s10_w11_prep_pipeline]].
