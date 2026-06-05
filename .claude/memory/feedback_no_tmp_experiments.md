---
name: No experiments in /tmp — use experiments/sNNN_<name>/
description: All host-side test/experiment scripts must live under examples/sentai_runtime/experiments/sNNN_<name>/, never in /tmp
type: feedback
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
All host-side test, smoke check, or one-off experiment scripts MUST live under `examples/sentai_runtime/experiments/sNNN_<name>/` (one folder per session, README + script(s) + outputs), NEVER in `/tmp`.

**Why:** `/tmp` is wiped on reboot, invisible to git, and makes prior sessions unreproducible. The operator wants experiments to accumulate as durable lab notes alongside the code, the way `s001`..`s085` already do. Convention is documented in `examples/sentai_runtime/experiments/README.md` and the methodology file next to it.

**How to apply:** When the user asks for any host-side script — radio test, flashing helper, telemetry capture, etc. — find the next free `sNNN` (currently `s086_radio_smoke` is the latest), create `experiments/sNNN_<name>/` with a `README.md` (what it proves, how to run, pass criteria) plus the scripts. If a previous version was put in `/tmp`, MOVE it before re-running. ON-BOARD `_t_*.py` diag drivers go under `examples/sentai_runtime/diag/` (different rule, same spirit — `_host_*` prefix marks files the uploader skips).
