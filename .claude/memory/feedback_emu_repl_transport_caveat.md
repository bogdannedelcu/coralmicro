---
name: feedback-emu-repl-transport-caveat
description: B8 ARM emulator uses LPUART6 for REPL only as test scaffolding — production REPL is USB CDC ACM; UART is reserved for Crazyflie CRTP.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8b2903f8-f695-46ef-9f13-edba1cccdfeb
---

Operator note 2026-06-02 (during B8.4 review):

> "noi am rulat f mult REPL peste USB, nu peste UART, zic asta pentru ca la
> un moment dat vom trece cu legatura Crazyflie - UART CRTP care va cam
> inlocui terminalul clasic si nu vom mai putea rula comenzi REPL pe acolo
> ... sa tinem cont de asta, ca e bine sa fim cat mai aproape de boardul
> real"

Rule for emulator work:

- The production SentAI REPL is **USB CDC ACM** on `/dev/ttyACM0`, not
  LPUART6.  `sentai.console("uart")` is an operator-driven switch, not the
  default.
- B8.3/B8.4 expose the REPL over LPUART6 because Renode's RT1176-like
  platform has a working `UART.NXP_LPUART` model and no USB CDC ACM
  endpoint.  This is a **test scaffolding compromise**, not the production
  REPL contract.
- Once LPUART/UART becomes the Crazyflie CRTP transport (planned),
  multiplexing an interactive REPL on the same UART will not work in the
  emulator either, and would be flat-out wrong on the real board.

**Why:** the thesis claim is that the emulator validates the same task/ISR
shape that runs on the real board.  Routing REPL through LPUART6 in the
emulator drifts the "console" boundary away from production and risks
hiding USB device-controller behavior we will eventually need to model.

**How to apply:**

- For B8.5+: treat the LPUART6 REPL as development convenience.  Drive any
  per-test Python through `import + run` against a baked or staged
  `mission.py`, **not** through long-lived interactive REPL bytes.
- Before any work that touches UART transport semantics (CRTP bridge,
  Crazyflie integration, link-level), confirm the emulator is not still
  pretending UART is the REPL.
- Two future emulator REPL options, in order of fidelity to the real
  board:
  1. model USB CDC ACM in Renode (custom peripheral, larger scope);
  2. drop interactive REPL entirely, mission-only.

Cross-refs: [[project_op_s10_w11_prep_pipeline]] (sentai runtime),
[[feedback_radio_no_file_transfer]] (CRTP MTU constraints).
