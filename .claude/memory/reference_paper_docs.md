---
name: Paper/ investigation docs added 2026-04-24
description: Models inventory + model perf plot + apex firmware RE + coral hostside findings — check these before re-investigating TPU/models questions
type: reference
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
Three standalone docs under `examples/sentai_runtime/paper/` added this session.
Read them before re-doing TPU/model investigations — they capture hours of work.

| Path | What it contains | When to consult |
|---|---|---|
| `paper/models.md` | Inventory of all 24 `models/*.tflite` — file sizes, per-invoke ins/input/params/output bytes, chunk counts, dma_hint breakdown | When choosing which model to deploy / comparing complexity |
| `paper/model_perf.png` + generator in `paper/models.md` | Estimated FPS ceiling per model from USB throughput (calibrated 180 MB/s input, 158 MB/s ins from real measurement) | When planning a new model integration or sanity-checking FPS claims |
| `paper/apex_fw.md` | Reverse engineering of `apex_latest_single_ep.bin` — 8051 machine code + USB descriptor table + DMA command codes at XRAM 0x801C | When asked "could we modify apex" or "is feature X in the firmware" |
| `paper/coral_hostside.md` | Linux-host measurements via pycoral, USB 3.0 vs 2.0 comparisons, verbose trace analysis of one invoke, all USB tunable flags discovered | When debating protocol-level TPU optimisations |

## Tools also added under `scripts/`

- `scripts/d51.py` — 8051 (MCS-51) disassembler for apex.bin
- `scripts/inspect_edgetpu_model.py` — parse .tflite edgetpu-custom-op, dump Package → MultiExecutable → Executable → instruction_bitstreams
- `scripts/inspect_all_models.py` — batch version, generates `paper/models.md` table
- `scripts/plot_model_perf.py` — generates `paper/model_perf.png`

All tools tested and working with `venv/bin/python` (Python 3.12 + tensorflow).
