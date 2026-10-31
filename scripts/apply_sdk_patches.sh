#!/usr/bin/env bash
# apply_sdk_patches.sh — apply the sentai patches sitting in
#   patches/coralmicro-rt1176-sdk/   onto   third_party/nxp/rt1176-sdk/
#   patches/micropython/             onto   third_party/micropython/
#
# Idempotent: uses `git apply --check` before applying; skips patches
# that are already present.  Fails loud if a patch fails to apply
# (context drift after submodule bump).  Meant to be called from CMake
# configure step AND from developers running `scripts/setup.sh`.
#
# Note on MicroPython: the ARM build compiles the embed-package copy under
# examples/sentai_runtime/micropython_embed/ (tracked in git), which the
# /qstr-regen recipe regenerates FROM the submodule.  Patching the
# submodule here keeps that regen from silently reverting the fix.
#
# Exit codes:
#   0  patches applied (or already present)
#   1  a patch failed to apply → manual resolution needed

set -u

# Resolve the repo root from this script's location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# patch-dir-name : submodule-path  (relative to repo root)
TARGETS=(
    "coralmicro-rt1176-sdk:third_party/nxp/rt1176-sdk"
    "micropython:third_party/micropython"
)

shopt -s nullglob
applied=0
skipped=0
failed=0

for t in "${TARGETS[@]}"; do
    PATCH_DIR="$REPO_ROOT/patches/${t%%:*}"
    SUBMODULE="$REPO_ROOT/${t#*:}"

    if [[ ! -d "$PATCH_DIR" ]]; then
        echo "[apply_sdk_patches] no patch dir at $PATCH_DIR — nothing to do"
        continue
    fi
    patches=("$PATCH_DIR"/*.patch)
    if (( ${#patches[@]} == 0 )); then
        echo "[apply_sdk_patches] no patches under $PATCH_DIR — nothing to do"
        continue
    fi
    if [[ ! -d "$SUBMODULE/.git" && ! -f "$SUBMODULE/.git" ]]; then
        echo "[apply_sdk_patches] submodule not initialised at $SUBMODULE"
        echo "[apply_sdk_patches] run: git submodule update --init --recursive"
        exit 1
    fi

    for p in "${patches[@]}"; do
        name="${t%%:*}/$(basename "$p")"
        # Already applied?  --reverse --check succeeds iff the patch is
        # already in the tree (i.e. applying the reverse would be a no-op).
        if git -C "$SUBMODULE" apply --reverse --check "$p" >/dev/null 2>&1; then
            echo "[apply_sdk_patches] skip (already applied): $name"
            skipped=$((skipped+1))
            continue
        fi
        # Fresh apply.  --check first so we fail before writing anything.
        if ! git -C "$SUBMODULE" apply --check "$p" >/dev/null 2>&1; then
            echo "[apply_sdk_patches] ERROR: $name does not apply cleanly"
            echo "  — submodule may have moved past the patch base commit."
            echo "  — inspect with: (cd $SUBMODULE && git apply --check -v $p)"
            failed=$((failed+1))
            continue
        fi
        if git -C "$SUBMODULE" apply "$p"; then
            echo "[apply_sdk_patches] applied: $name"
            applied=$((applied+1))
        else
            echo "[apply_sdk_patches] ERROR: $name apply failed"
            failed=$((failed+1))
        fi
    done
done

echo "[apply_sdk_patches] summary: applied=$applied skipped=$skipped failed=$failed"
exit $(( failed > 0 ? 1 : 0 ))
