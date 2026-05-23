---
name: external-repo-patch-log
description: Maintain a centralized journal of patches applied to external repositories (CrazySim, PX4, Gazebo plugins, vendor SDKs) so reverts are traceable and future sessions know what's intentional. Use when modifying any file outside /home/bogdan/work/coralmicro/.
---

# /external-repo-patch-log

This project touches multiple external repos that don't live in the coralmicro git tree:
- `/home/bogdan/work/crazyflie/CrazySim/...` — CrazySim cf2 SITL + Gazebo model SDFs
- `/home/bogdan/work/...` — possibly PX4 SITL when OP-S10-W9 lands
- `third_party/nxp/rt1176-sdk/` — vendor SDK (patched via `scripts/apply_sdk_patches.sh`; tracked)

For the SDK, patches are in `patches/coralmicro-rt1176-sdk/` and re-applied idempotently — already disciplined.

For everything ELSE (CrazySim, PX4 when it lands), the convention has been `.pre_<reason>_<date>` backup files and ad-hoc memory entries.  This is fragile — yesterday's session can't tell whether `model.sdf.jinja.pre_no_cheat_20260517` is the "before" or "after", or whether the live `model.sdf.jinja` was reverted afterwards.

## The journal — `ideas/external_patches.md`

Single source of truth.  Maintained as a chronological log, append-only, English.  Each entry:

```markdown
## YYYY-MM-DD <WBS-code> — <one-line summary>

**Repo**: <absolute path>  (`<git-remote-if-applicable>`)
**File(s)**: <path within that repo>
**Reason**: <why we patched — link to WBS + auto-memory>
**Patch type**: edit / deletion / addition / config-change
**Backup**: <path to .pre_* if created; or "git history at commit <sha>" if relying on git>
**Revert command**:
\`\`\`bash
# Concrete one-liner to undo this patch
\`\`\`
**Status**: applied / reverted / partially-reverted / canonical-now
**Notes**: <anything non-obvious>
```

## When to use this skill

Whenever you modify a file outside `/home/bogdan/work/coralmicro/`.  Even a one-line change.

## Recipe

### Step 1 — Make the change with a backup

```bash
FILE=/home/bogdan/work/crazyflie/CrazySim/.../model.sdf.jinja
REASON=no_cheat
DATE=$(date +%Y%m%d)
cp "$FILE" "${FILE}.pre_${REASON}_${DATE}"
# now edit $FILE
```

The `.pre_<reason>_<date>` filename pattern is the convention.

### Step 2 — Log the patch

Append to `ideas/external_patches.md`:

```bash
cat >> /home/bogdan/work/coralmicro/ideas/external_patches.md <<EOF

## $(date +%Y-%m-%d) <OP-Sx-Wy> — <one-line>

**Repo**: $(realpath ${FILE%/*})
**File**: $(basename $FILE)
**Reason**: <why>
**Patch type**: <edit/deletion/...>
**Backup**: ${FILE}.pre_${REASON}_${DATE}
**Revert command**:
\`\`\`bash
cp ${FILE}.pre_${REASON}_${DATE} ${FILE}
\`\`\`
**Status**: applied
**Notes**: <anything>

EOF
```

### Step 3 — Commit the log update to coralmicro (the external repo itself is not touched by coralmicro git)

```bash
cd /home/bogdan/work/coralmicro
git add ideas/external_patches.md
# (will be committed with the WBS-coded commit that motivated the patch)
```

## When to status-update an entry

- After reverting → change Status to `reverted` + add date.
- When the patch becomes the canonical state we want forever → change Status to `canonical-now` + note who upstreamed it (or "not upstreamed; we live with the local patch").
- When the patch is partially reverted → status `partially-reverted` + describe what remains.

## Reading the log

Before any session that touches external repos, scan the log:

```bash
grep -A 8 -E '^## [0-9]{4}' /home/bogdan/work/coralmicro/ideas/external_patches.md \
    | grep -E '(Status|File|Repo)' | head -30
```

Catches "did we forget to revert that cheat plugin disable?" type questions.

## The CrazySim cheat-plugin entry (canonical seed)

If `ideas/external_patches.md` doesn't exist yet, seed with this entry from the OP-S8-W1 history:

```markdown
# External repository patch log

Chronological journal of patches applied OUTSIDE the coralmicro git tree.
Append-only.  English only.  Format: see `[[external-repo-patch-log]]` skill.

## 2026-05-17 OP-S8-W1-T1 — disable gz-sim-odometry-publisher cheat plugin

**Repo**: /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie
**File**: model.sdf.jinja
**Reason**: Crisis OP-S8-W1 — plugin was injecting GT pose into cf2 EKF as CrtpExtPose, invalidating every prior cf2-side drift number.  See [[cf2-sitl-cheat-odom-gt]] + [[op-s8-w1-cf2-sim-honest]].
**Patch type**: deletion (commented out <plugin> block)
**Backup**: model.sdf.jinja.pre_no_cheat_20260517
**Revert command**:
\`\`\`bash
cp /home/bogdan/work/crazyflie/CrazySim/.../model.sdf.jinja.pre_no_cheat_20260517 \
   /home/bogdan/work/crazyflie/CrazySim/.../model.sdf.jinja
\`\`\`
**Status**: canonical-now (the cheat must STAY disabled per anti-cheat rule)
**Notes**: DO NOT revert.  If you find the cheat re-enabled, restore THIS patch.
```

## Reject patterns

- Editing an external file with no `.pre_*` backup AND no log entry → unrecoverable without git history.
- Logging the patch but skipping the backup → revert command relies on memory.
- Backups in `/tmp` → wiped on reboot per `[[no-tmp-experiments]]`.
- Multiple `.pre_*` files for the same file with no log → ordering unclear.

## See also

- `embeded.md` §6.5 (configuration management — every artifact identified)
- `embeded.md` §6.2 (SOUP — version pinning + patch trail)
- `[[op-s8-w1-cf2-sim-honest]]` auto-memory — the cheat plugin crisis
- `[[sim-precondition-gate]]` skill — checks `.pre_*` orphans as a signal
- `patches/coralmicro-rt1176-sdk/` — analog discipline for the vendor SDK (already in place)
