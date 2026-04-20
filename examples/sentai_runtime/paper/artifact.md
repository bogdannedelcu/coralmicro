# Artifact description and data availability

An MDPI-structured research article must state where its data and
code live and how a reader can reproduce the reported results.  This
chapter provides that statement plus the mapping from each numeric
claim in the paper to the raw per-iteration CSV that supports it.

---

## 1. Data availability statement

All data presented in this study are openly archived alongside the
source code of the measurement framework in this repository.  The
raw per-iteration CSVs for every experiment are under
[`../experiments/`](../experiments/), mirrored byte-for-byte from the
device's LittleFS `/diags/` tree at the time of capture.  Statistical
summaries in the paper tables are derived offline by the generator
script [`../experiments/_build_appendix.py`](../experiments/_build_appendix.py),
which operates solely on the checked-in CSVs and reproduces every
appendix number without access to the hardware.

No proprietary, subject-private, or licence-restricted data is
involved.

---

## 2. Repository layout

At repository root, the subtrees that together constitute the
reported artifact are:

| Path | Purpose |
|---|---|
| `examples/sentai_runtime/` | Firmware application source (`sentai_runtime.cc`, `modsentai_*.c`, `detection_task.cc`, `sentai_httpd.cc`, `sentai_lfs_task.cc`, …) |
| `examples/sentai_runtime/diag/` | MicroPython diagnostic package — experiments E1-E18 |
| `examples/sentai_runtime/diag/_host_upload_repl.py` | Host-side REPL uploader used to push `diag/*.py` to the board |
| `examples/sentai_runtime/diag/drivers/` | Host-side REPL drivers (`_e18_post_refactor.py`, …) that invoke experiments end-to-end |
| `examples/sentai_runtime/experiments/` | The archived session data presented in this paper |
| `examples/sentai_runtime/paper/` | Paper source (this file and siblings) |
| `examples/sentai_runtime/agent/` | Operating documentation (embeded.md discipline rules, agent.md handoff guide) |
| `libs/base/gpio.cc`, `libs/camera/`, `libs/base/filesystem.cc`, … | SentAI platform libraries modified as part of this work |
| `third_party/nxp/rt1176-sdk/…` | Vendor SDK, used as-is except for documented table entries |
| `third_party/micropython/` | MicroPython embed port, used as-is |
| `scripts/flashtool.py` | NXP flashing tool used to program the board |

A reader who wishes to reproduce the measurements in this paper needs
the three subtrees `examples/sentai_runtime/*` plus the cross-cutting
library changes in `libs/camera/cam_mux.h` and `libs/base/gpio.cc`,
all of which are under the repository's open-source licence.

---

## 3. Firmware build identification

The firmware image presented as the "Fix B post-review" configuration
— responsible for every numeric claim in Tables 3, 4, 6 and 7 of
[evaluation.md](evaluation.md) — is:

| Field | Value |
|---|---|
| Build number | 640 |
| Build timestamp | stored in `build_version.h` and echoed at every boot: `SentAI build #640 (2026-04-20 …)` |
| Git branch | `feature/ov5640-camera-support` |
| Full toolchain | CMake 3.x + Ninja, `arm-none-eabi-gcc` 10.3.x, NXP MCUXpresso SDK vendored under `third_party/nxp/rt1176-sdk/` |
| Reconstruction | `cmake -S . -B build && cmake --build build --target sentai_runtime` |
| Flash | `python3 scripts/flashtool.py -e sentai_runtime` (persistent; omit `--ram`) |

Earlier builds (#622 for the first eDMA measurement; #632 for Fix A;
#635 for Fix B pre-refactor) are reachable by checking out earlier
commits of the same branch and rebuilding.  The cross-session
reproducibility table (Table 4 of [evaluation.md](evaluation.md))
documents the three builds that together demonstrate the refactor is
performance-neutral.

---

## 4. Hardware requirements

| Requirement | Minimum |
|---|---|
| MCU board | Coral Dev Board Micro with SentAI daughter-board v1.0 |
| USB cable + host | one USB-C cable to a Linux host (CDC-ACM + CDC-NCM required; most host USB stacks supply these out of the box) |
| Scene | static scene with either no target class or a fixed target count; the paper reports a zero-target scene — see [experimental_setup.md](experimental_setup.md) §5 |
| Bench time | ≈ 5 s per experiment sweep; ≈ 30 s per session including begin/end/snapshots; the entire reported E18 benchmark is ≈ 15 s on-device |

No special test equipment (oscilloscope, logic analyser, thermal
chamber) is required to reproduce any number in this paper.  That is
intentional: the measurement framework captures every stage it needs
internally and emits CSVs.

---

## 5. Reproducing a single session from scratch

Assuming the repository is checked out at the branch and the board
is connected:

```bash
# 1. Build + flash
cd /path/to/coralmicro
cmake -S . -B build
cmake --build build --target sentai_runtime
python3 scripts/flashtool.py -e sentai_runtime

# 2. Wait for the board to boot (up to ~15 s first time after flash)
for i in $(seq 1 20); do
  curl -s -m 2 -o /dev/null http://10.0.0.1/ && break
  sleep 1
done

# 3. Push the latest diag/ package
cd examples/sentai_runtime
python3 diag/_host_upload_repl.py \
    --file e_pipeline.py --file _util.py \
    --file _session.py --file __init__.py

# 4. Execute the reported head-to-tail benchmark
python3 diag/drivers/_e18_post_refactor.py

# 5. Fetch the session folder (session id is the next free sNNN on
#    this board's counter; inspect /api/ls/diags/ to find it)
curl -s http://10.0.0.1/api/ls/diags/ | python3 -m json.tool | less

# 6. Pull the three CSVs
for f in 001_e18_A_fixed_cam0.csv 002_e18_B_fixed_cam1.csv \
         003_e18_C_alt_cam0_cam1.csv; do
    curl -s http://10.0.0.1/api/raw/diags/sNNN_e18_post_refactor/$f > $f
done
```

The numbers from step 6 should agree with Table 3 of
[evaluation.md](evaluation.md) within the ≤ 1 ms cross-session noise
documented in Table 4.

---

## 6. Claim-to-file map (the "which CSV supports which table" reference)

The table below is the single authoritative map from every numeric
claim in this paper to its on-disk evidence.  Rows are ordered by
where the claim appears in the paper.

**Table 1.** Paper-level claim-to-evidence index.

| Claim | Where it appears | Evidence (folder + file) | Generator |
|---|---|---|---|
| CPU memcpy ≈ 24 ms baseline | [memcpy.md](memcpy.md) §"Baseline", [evaluation.md](evaluation.md) Table 2 | [memcpy.md CSV block](memcpy.md) (10-run table is reproduced inline); raw per-iteration CSVs captured by [`../_e15_ab.py`](../_e15_ab.py) during an A/B session, archived inline in the paper chapter | inline |
| eDMA memcpy ≈ 14.6 ms | [memcpy.md](memcpy.md) §"Optimised", [evaluation.md](evaluation.md) Table 2 | same A/B session as above | inline |
| 15.47 FPS post-eDMA sustained, 512×512 | [memcpy.md](memcpy.md) §"Per-run results" optimised block; [evaluation.md](evaluation.md) Table 1 | [`../experiments/s032_e15_x20/`](../experiments/s032_e15_x20/) (20 × 20 frames, post-eDMA) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix C |
| 13.39 FPS pre-eDMA | [memcpy.md](memcpy.md) §"Baseline" | same A/B session, baseline block | inline |
| E14 baseline 6.4 FPS (80-class model) | [evaluation.md](evaluation.md) §8 "Each optimisation exposes the next"; Appendix B | [`../experiments/s021_e14_x20/`](../experiments/s021_e14_x20/), [s022](../experiments/s022_e14_x20/), [s023](../experiments/s023_e14_x20/), [s025](../experiments/s025_e14_x20/), [s031](../experiments/s031_e14_x20/) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix B |
| Pre-Fix A alternating FPS 4.7, asymmetry +64 ms | [cam_switch.md](cam_switch.md) §"E15 vs E16 — head-to-head"; [evaluation.md](evaluation.md) Table 7 | [`../experiments/s034_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv`](../experiments/s034_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix D |
| Fix A null result (asymmetry unchanged) | [cam_switch.md](cam_switch.md) §"Fix A"; [evaluation.md](evaluation.md) Table 7 | [`../experiments/s035_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv`](../experiments/s035_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix D |
| Fix B alternating FPS 6.87, asymmetry ≤ 1 ms (E16 alone) | [cam_switch.md](cam_switch.md) §"Fix B — flip-on-EOF" | [`../experiments/s042_e16_eof_30fps_x40/001_e16_camswitch_cam0_cam1.csv`](../experiments/s042_e16_eof_30fps_x40/001_e16_camswitch_cam0_cam1.csv) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix E |
| E18 head-to-tail, pre-refactor (s043) | [cam_switch.md](cam_switch.md) §"Head-to-tail benchmark" | [`../experiments/s043_e18_headtail_drain2/{001,002,003}_e18_*.csv`](../experiments/s043_e18_headtail_drain2/) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix G |
| E18 head-to-tail, post-refactor (s045) | [cam_switch.md](cam_switch.md) §"Session `s045…`"; [evaluation.md](evaluation.md) Table 3 | [`../experiments/s045_e18_post_refactor/{001,002,003}_e18_*.csv`](../experiments/s045_e18_post_refactor/) | [`_build_appendix.py`](../experiments/_build_appendix.py) §Appendix G |
| Cross-session reproducibility ≤ 1 ms | [evaluation.md](evaluation.md) Table 4 | [`../experiments/s043_e18_headtail_drain2/`](../experiments/s043_e18_headtail_drain2/), [`s044`](../experiments/s044_e18_headtail_drain2/), [`s045`](../experiments/s045_e18_post_refactor/) | derivation in [statistical_notes.md](statistical_notes.md) |
| Pre-fix mid-buffer seam (visual) | [cam_switch.md](cam_switch.md) §"Visual evidence"; [evaluation.md](evaluation.md) Table 5 | [`../experiments/s038_e17_drain_ab/e17_t1_frames/{002_cam0_133ms,003_cam1_202ms}.jpg`](../experiments/s038_e17_drain_ab/e17_t1_frames/) | visual inspection |
| Post-fix seam-free at drain=2 (visual) | [cam_switch.md](cam_switch.md) §"Visual evidence"; [evaluation.md](evaluation.md) Table 5 | [`../experiments/s041_e17_eof_check/e17_t2_frames/`](../experiments/s041_e17_eof_check/e17_t2_frames/) | visual inspection |
| drain=1 residual artefacts | [cam_switch.md](cam_switch.md) §"Known limitations"; [threats_to_validity.md](threats_to_validity.md) §2.5 | [`../experiments/s041_e17_eof_check/e17_t1_frames/`](../experiments/s041_e17_eof_check/e17_t1_frames/) | visual inspection |
| Fault counters zero on nominal run | [evaluation.md](evaluation.md) Table 6; [cam_switch.md](cam_switch.md) §"Shipping runtime surface" | `sentai.diag.cam_stats()` output at end of sessions `s043`, `s044`, `s045` | [modsentai_diag.c:mod_sentai_diag_cam_stats](../modsentai_diag.c) |

---

## 7. Data integrity check

A reviewer can verify the integrity of the archived CSVs against
paper tables with three commands:

```bash
# 1. All appendix tables re-derived from raw CSVs (no board access)
cd examples/sentai_runtime
python3 experiments/_build_appendix.py  # overwrites experiments/_appendix section of README

# 2. The specific E18 head-to-tail claim
python3 - <<'EOF'
import csv, statistics
def mean(csvpath, col):
    rows = list(csv.DictReader(open(csvpath).readlines()))[1:]  # drop warmup
    return statistics.mean(int(r[col]) for r in rows)
for sess in ('s043_e18_headtail_drain2', 's044_e18_headtail_drain2', 's045_e18_post_refactor'):
    for (label, f) in (('A', '001_e18_A_fixed_cam0.csv'),
                       ('B', '002_e18_B_fixed_cam1.csv'),
                       ('C', '003_e18_C_alt_cam0_cam1.csv')):
        m = mean('experiments/%s/%s' % (sess, f), 'total_frame_ms')
        print('%s %s mean=%.1f fps=%.2f' % (sess, label, m, 1000/m))
EOF

# 3. Visual evidence: open the two representative frames and compare
xdg-open experiments/s038_e17_drain_ab/e17_t1_frames/002_cam0_133ms.jpg
xdg-open experiments/s041_e17_eof_check/e17_t1_frames/003_cam1_201ms.jpg
```

---

## 8. Author contributions, funding, conflicts (MDPI boilerplate)

*(Placeholders.  These will be filled by the authors at submission.
Included here so the paper source already has the MDPI-required
declarations in a canonical location, rather than being scattered
across templates.)*

- **Author Contributions**: Conceptualization, *TBD*; methodology,
  *TBD*; software, *TBD*; validation, *TBD*; formal analysis,
  *TBD*; investigation, *TBD*; resources, *TBD*; data curation,
  *TBD*; writing — original draft preparation, *TBD*; writing —
  review and editing, *TBD*; visualization, *TBD*; supervision,
  *TBD*; project administration, *TBD*; funding acquisition,
  *TBD*.  All authors have read and agreed to the published version
  of the manuscript.
- **Funding**: *This research received no external funding.* (or
  list the specific grant).
- **Institutional Review Board Statement**: *Not applicable.* (This
  study involves no human subjects, no animal subjects, and no
  identifying data — it is an embedded-systems measurement study on
  a development board.)
- **Informed Consent Statement**: *Not applicable.*
- **Data Availability Statement**: All data presented in this study
  are available in the repository accompanying this paper under
  `examples/sentai_runtime/experiments/`, along with the generator
  script that reproduces every summary statistic from the raw
  per-iteration CSVs.  See §1 above for details.
- **Acknowledgments**: *TBD* — list any non-funding support
  (infrastructure, advice, early reviewers).
- **Conflicts of Interest**: *The authors declare no conflicts of
  interest.* (or as applicable).

---

## 9. Supplementary materials

Following MDPI's supplementary-materials convention, these artefacts
accompany the main paper but are not required to read the narrative:

- `experiments/README.md` — per-session narrative index + appendices
  A–G with per-experiment tables derived from every CSV in this
  archive.
- `experiments/methodology.md` — full measurement protocol
  (warm-up, drop-first-sample convention, reproducibility rules,
  noise budget).
- `experiments/_build_appendix.py` — offline appendix generator; runs
  on the CSVs in this archive, produces the appendix markdown with
  no board access.
- `agent/agent.md` — operational handoff guide (how to pick up this
  project, how to drive the REPL, how to regenerate QSTRs, how to
  warm-reset a stuck REPL).
- `agent/embeded.md` — NASA/JPL-style coding-discipline rules used
  during the firmware refactor phase.

All of the above are in-repository, tracked by git, and version-
aligned with the firmware that produced the reported measurements.
