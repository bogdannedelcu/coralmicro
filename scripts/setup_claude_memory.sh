#!/usr/bin/env bash
# Wire ~/.claude/projects/<derived>/memory -> .claude/memory in this repo.
#
# Claude Code derives the per-project memory path from the absolute working
# directory by replacing every '/' with '-' (so /home/foo/coralmicro becomes
# -home-foo-coralmicro). That path is host-specific, so the symlink is NOT
# checked in — recreate it on every fresh clone / new machine.
#
# Idempotent: re-running after the symlink exists is a no-op.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MEMORY_SRC="$REPO_DIR/.claude/memory"

if [[ ! -d "$MEMORY_SRC" ]]; then
    echo "ERROR: $MEMORY_SRC does not exist — wrong repo or memory not yet tracked." >&2
    exit 1
fi

# Derive Claude's per-project slug from the repo absolute path.
SLUG="${REPO_DIR//\//-}"   # /home/foo/coralmicro -> -home-foo-coralmicro
LINK_DIR="$HOME/.claude/projects/$SLUG"
LINK_PATH="$LINK_DIR/memory"

mkdir -p "$LINK_DIR"

if [[ -L "$LINK_PATH" ]]; then
    current="$(readlink "$LINK_PATH")"
    if [[ "$current" == "$MEMORY_SRC" ]]; then
        echo "OK: $LINK_PATH already points to $MEMORY_SRC"
        exit 0
    fi
    echo "ERROR: $LINK_PATH is a symlink to $current (expected $MEMORY_SRC)." >&2
    echo "Remove it manually if stale, then re-run." >&2
    exit 1
fi

if [[ -e "$LINK_PATH" ]]; then
    echo "ERROR: $LINK_PATH exists and is not a symlink." >&2
    echo "Move its contents into $MEMORY_SRC (commit them) and remove the original, then re-run." >&2
    exit 1
fi

ln -s "$MEMORY_SRC" "$LINK_PATH"
echo "Linked $LINK_PATH -> $MEMORY_SRC"
echo "Claude will now load this repo's tracked memory for project slug: $SLUG"
