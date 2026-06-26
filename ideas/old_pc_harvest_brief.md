# Briefing — Harvest SentAI external dependencies into git (run on the OLD PC)

**Audience:** an autonomous coding agent running on the OLD computer (the one
with the fully working SentAI simulation/emulation environment).

**One-line goal:** make the `coralmicro` git repository *self-sufficient* —
everything needed to rebuild the SentAI sim/emulator environment from scratch
must be committed and pushed, so a fresh machine can reproduce it from git alone.

---

## 1. The problem (context — read first)

We are bringing the SentAI project up on a NEW Linux machine. There we:

- checked out branch `integration/from-180bbb5f` (newest, commit 2026-06-05);
- pulled all 25 git submodules recursively (OK);
- confirmed the ARM toolchain auto-downloads at CMake configure time (OK).

The **firmware/build is fully in git**. The **simulation + emulation environment
is NOT.** It lives in external repos under `~/work/` that are *not* part of the
`coralmicro` git tree:

- `~/work/crazyflie/CrazySim/` — cf2 SITL + Gazebo worlds/models/plugins
- `~/work/crazyflie/CrazySim/crazyflie-firmware/` — cf2 firmware (patched)
- `~/work/px4/PX4-Autopilot/` — PX4 SITL + a SentAI airframe/model
- `~/work/renode_portable/` — Renode binary
- `~/work/libedgetpu/`, `~/work/edge/edgetpu/` — EdgeTPU host driver + artifacts
- a distrobox container named `crazysim-garden` (Ubuntu 22.04 + Gazebo Garden 7.9)

The in-repo installers (`sim/scripts/install_crazysim.sh`,
`install_gazebo_harmonic.sh`, `install_px4_sitl.sh`) clone the **upstream**
versions of these. But the project applied **SentAI-specific modifications** by
hand directly in those external trees. Those modifications exist **only on this
old PC**. Reinstalling upstream on the new machine silently loses them.

The repo *documents* most of these patches in prose
(`ideas/external_patches.md`, `Sim.md`) but does **not** contain the actual
patched files or machine-applyable diffs. So the environment is currently **not
reproducible from git alone.** Your job is to close that gap.

---

## 2. What you are looking for (suspected gaps)

These are the known SentAI-specific deviations from upstream. Capture them — but
do **not** treat the list as exhaustive: capture the *full* delta of each
external git repo vs its upstream (step 4) so nothing is missed.

| # | External path (relative to its repo root) | What it is | Kind |
|---|---|---|---|
| 1 | `tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf` | WhyCon world actually flown in s219/s229/s233; 7-marker pad incl. asymmetric marker `N` | new file |
| 2 | `tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf` | CRTP-only world | new file |
| 3 | `tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja` | "no-cheat" patch (gz-sim-odometry-publisher disabled) | edit |
| 4 | `tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp` | baro subscription disabled | edit |
| 5 | `crazyflie-firmware: src/hal/src/socketlink.c` | ASSERT→drop+count fix (local commit `40d34708`) | edit/commit |
| 6 | `crazyflie-firmware: src/modules/src/estimator/estimator_kalman.c` | `KALMAN_USE_BARO_UPDATE` disabled | edit |
| 7 | PX4: `x500_sentai` model + matching airframe/autostart file | SentAI PX4 model | new files |

Cross-check what you capture against the prose in
`ideas/external_patches.md` and `Sim.md`. **Flag any captured delta that is NOT
already documented there** — that is the most valuable thing to surface.

Also note `.pre_<reason>_<date>` backup files next to several of these — they are
the convention for "before-patch" snapshots. Keep them out of the vendored diffs
(they are noise), but mention their presence.

---

## 3. Part A — first verify the `coralmicro` repo itself is fully pushed

Before harvesting external stuff, make sure the old PC isn't sitting on
unpushed/uncommitted work in the coralmicro repo itself (newer experiments,
todo updates, agent.md edits, etc.).

```bash
cd ~/work/coralmicro
git status --porcelain          # any uncommitted changes?
git branch --show-current       # which branch?
git log --oneline origin/$(git branch --show-current)..HEAD 2>/dev/null   # local-only commits?
git submodule status            # any submodule with local changes (+ / - prefix)?
```

If there are uncommitted changes or local-only commits on the integration
branch, **commit and push them** (after a quick sanity review). Report exactly
what you pushed. This alone may be a large part of "the repo is incomplete."

---

## 4. Part B — harvest external SentAI deltas into the repo

Create a vendoring area in the coralmicro repo and capture, for each external
git repo, (a) its identity + pinned commit, (b) a single patch of ALL local
deviations from upstream, and (c) any wholly-new SentAI asset files.

```bash
cd ~/work/coralmicro
mkdir -p sim/vendor_patches/{crazysim,crazyflie-firmware,px4,manifest}
```

For each external repo, run this capture pattern (adjust REPO + OUT):

```bash
capture() {
  local REPO="$1" OUT="$2"
  echo "## $REPO"  > "$OUT/INFO.txt"
  git -C "$REPO" remote -v               >> "$OUT/INFO.txt"
  echo "HEAD: $(git -C "$REPO" rev-parse HEAD)" >> "$OUT/INFO.txt"
  git -C "$REPO" log --oneline -15       >> "$OUT/INFO.txt"
  # local commits not in upstream:
  git -C "$REPO" log --oneline @{upstream}..HEAD 2>/dev/null >> "$OUT/INFO.txt" || \
    git -C "$REPO" log --oneline origin/HEAD..HEAD 2>/dev/null >> "$OUT/INFO.txt"
  # FULL delta vs upstream (committed local commits + uncommitted working tree):
  git -C "$REPO" diff @{upstream} 2>/dev/null > "$OUT/delta_vs_upstream.patch" || \
    git -C "$REPO" diff origin/HEAD > "$OUT/delta_vs_upstream.patch"
  # untracked files (the new SentAI assets show up here):
  git -C "$REPO" status --porcelain --untracked-files=all > "$OUT/status.txt"
}

capture ~/work/crazyflie/CrazySim                  sim/vendor_patches/crazysim
capture ~/work/crazyflie/CrazySim/crazyflie-firmware sim/vendor_patches/crazyflie-firmware
capture ~/work/px4/PX4-Autopilot                   sim/vendor_patches/px4
```

Then copy the wholly-new SentAI asset files (the `??` untracked entries in
`status.txt` plus the worlds/models above) into the vendor area, preserving
relative paths, e.g.:

```bash
# CrazySim worlds + model + plugin (copy the live files, not the .pre_* backups)
GZ=~/work/crazyflie/CrazySim/tools/crazyflie-simulation/simulator_files/gazebo
mkdir -p sim/vendor_patches/crazysim/files/worlds sim/vendor_patches/crazysim/files/models/crazyflie sim/vendor_patches/crazysim/files/plugins/CrazySim
cp "$GZ/worlds/sentai_whycon_small.sdf"  sim/vendor_patches/crazysim/files/worlds/
cp "$GZ/worlds/sentai_crazysim.sdf"      sim/vendor_patches/crazysim/files/worlds/
cp "$GZ/models/crazyflie/model.sdf.jinja" sim/vendor_patches/crazysim/files/models/crazyflie/
cp "$GZ/plugins/CrazySim/crazysim_plugin.cpp" sim/vendor_patches/crazysim/files/plugins/CrazySim/

# PX4 SentAI model + airframe (find exact paths first):
find ~/work/px4/PX4-Autopilot -iname '*x500_sentai*' -o -iname '*sentai*airframe*'
# then cp the matches under sim/vendor_patches/px4/files/ preserving structure
```

### Also capture the environment "shape" (versions to pin)

Write `sim/vendor_patches/manifest/versions.txt` with:

```bash
{
  echo "renode: $(~/work/renode_portable/renode --version 2>/dev/null | head -1)"
  echo "gz (host): $(gz sim --versions 2>/dev/null | head -1)"
  echo "distrobox list:"; distrobox list 2>/dev/null
  echo "crazysim-garden gz: $(distrobox enter crazysim-garden -- gz sim --versions 2>/dev/null | head -1)"
  echo "cflib: $(~/work/crazyflie/.venv/bin/python -c 'import cflib,os;print(os.path.dirname(cflib.__file__))' 2>/dev/null)"
  echo "cflib repo HEAD: $(git -C ~/work/crazyflie/crazyflie-lib-python rev-parse HEAD 2>/dev/null)"
  echo "px4 version: $(cat ~/work/px4/PX4-Autopilot/version.txt 2>/dev/null)"
} > sim/vendor_patches/manifest/versions.txt
```

If the `crazysim-garden` distrobox was created from a script or a one-liner,
capture that command into `sim/vendor_patches/manifest/distrobox_recipe.md`
(check shell history: `history | grep -i 'distrobox create'`). If you can't find
it, dump the base image: `distrobox list` + `podman inspect crazysim-garden`.

### libedgetpu (lower priority — note, don't necessarily vendor the .so)

```bash
{
  echo "libedgetpu repo HEAD: $(git -C ~/work/libedgetpu rev-parse HEAD 2>/dev/null)"
  git -C ~/work/libedgetpu remote -v 2>/dev/null
  echo "apex fw present: $(ls -la ~/work/libedgetpu/driver/usb/apex_latest_single_ep.bin 2>/dev/null)"
} >> sim/vendor_patches/manifest/versions.txt
```
Don't commit large binary build artifacts (`libedgetpu.so.1`) — record how to
rebuild instead. The `apex_latest_single_ep.bin` firmware blob ships with the
libedgetpu source, so the repo HEAD sha is enough.

---

## 5. Commit + push

```bash
cd ~/work/coralmicro
git add sim/vendor_patches
# (and any Part-A changes you committed separately)
git commit -m "vendor SentAI external sim deps (CrazySim worlds/plugin, cf2 fw patches, PX4 model) for git-reproducible env"
git push
```

Branch: same integration branch the repo is on (`integration/from-180bbb5f`
unless the old PC is on something newer — if so, say which).

---

## 6. Report back (what the new-PC side needs to know)

Produce a short summary answering:

1. Was the coralmicro repo itself fully pushed, or did you have to commit/push
   local work? What commits/files?
2. Which external repos had local deltas? Paste each `delta_vs_upstream.patch`
   size + the local-commit list.
3. **Which captured deltas were NOT already documented in
   `ideas/external_patches.md` / `Sim.md`?** (most important)
4. Exact PX4 SentAI model/airframe paths you found.
5. The pinned versions (Renode, Gazebo Garden in distrobox, PX4, cflib,
   libedgetpu) from `versions.txt`.
6. Anything referenced by scripts under `~/work/...` that you could NOT find on
   disk (i.e., a dangling dependency).

That report + the pushed `sim/vendor_patches/` tree is what makes the repo
verifiably complete.
