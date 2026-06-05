---
name: feedback-emu-tpu-must-run-real-edgetpu-manager
description: "HARD RULE — emulator TPU path must run the real libs/tpu/edgetpu_manager.cc + libs/usb/usb_host_task.cc on the emulated CM7, with USB packets passed through to the host's physical Coral. NOT a host bridge that replaces edgetpu_manager."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8b2903f8-f695-46ef-9f13-edba1cccdfeb
---

Operator note 2026-06-02 (during B8.8 first attempt):

> "eu as fi vrut ca freertos sa vorbeasca direct cu usb-ul... sa nu
> intermediem nimic cu pycoral. avem codul sursa scris in
> edgetpumanager"

Then clarified:

> "chipul RT1170 are un port usb dedicat prin care e legat la procesorul
> EdgeTpu un usb 2.0, acum noi in emulator vreau sa folosim ceva
> similar... adica sa verificam cum se initializeaza acel EdgeTpuManager
> in codul sursa coral micro si sa il legam la USB-ul virtual din host"

HARD RULE for any emulator TPU work:

- The emulated CM7 must run the **real** `libs/tpu/edgetpu_manager.cc`,
  `libs/tpu/edgetpu_task.cc`, `libs/tpu/edgetpu_driver.cc`,
  `libs/tpu/edgetpu_dfu_task.cc`, and the underlying NXP USB host stack
  (`libs/usb/usb_host_task.cc` + `usb_host_ehci.c`).
- The Renode platform model must wire RT1176 USB OTG2 at `0x4042C000`
  (the controller used for Coral on the SentAI board) to either:
    1. a Renode USB host controller model with USB device passthrough to
       the host's physical Coral (via libusb / USBIP), OR
    2. a Renode-side fake EdgeTPU device that implements the Coral
       protocol (DFU + runtime endpoints) so the real driver thinks it
       is talking to silicon.
- A host-side "mailbox" bridge that calls pycoral on the host **bypasses
  the entire edgetpu_manager + USB host stack** and is therefore the
  WRONG architecture for the emulator path.  Such a bridge is acceptable
  only as a debug shim explicitly labelled "not the production code
  path".
- An early B8.8 attempt at the mailbox approach (Renode `vtpu`
  peripheral + `vtpu_helper.py` calling pycoral) was reverted and is
  not preserved on disk.

Why this matters: the whole point of the emulator path is to run the
same code that runs on silicon, so emulator results are predictive of
on-board behavior.  A mailbox bypass would let the emulator pass while
the production firmware hits a real USB / DFU / driver bug.

How to apply:

- Any emu TPU gate must link the real edgetpu_manager + usb_host_task
  translation units into the firmware image.
- Renode platform changes go in `emu/renode/sentai_rt1176.repl`; any
  fork of Renode goes under `patches/` per the
  `external-repo-patch-log` policy.
- "Transparent USB Coral access from the emulator" is the goal.
  Partial work (firmware boots, hits first missing MMIO bit) is
  acceptable as a research milestone and should be documented in the
  B8 doc with the exact failure address / register name.

Cross-refs:
- [[feedback_emu_must_not_reuse_production_task_names]] (same spirit:
  emu must align with production semantics, not pretend);
- [[feedback_emu_pixel_format_caveat]] (don't conflate emu shortcuts
  with production format);
- `examples/sentai_runtime/agent/agent.md` §11 USB host stack notes.
