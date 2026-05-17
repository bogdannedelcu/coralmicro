#!/bin/bash
# ai_distill.sh — distill a file via local Ollama (qwen3-coder:30b on hpc.lan).
#
# Use this when you need the ESSENCE of a long file but don't want to
# burn tokens reading the whole thing into the main agent's context.
# Output goes to stdout; the agent reads only the distilled summary.
#
# Usage:
#   bash scripts/ai_distill.sh <file> "<question or instruction>"
#   bash scripts/ai_distill.sh --raw "<full prompt>"     # ad-hoc, no file
#
# Examples:
#   bash scripts/ai_distill.sh ideas/objects_plan.md \
#       "list the 8 in-scope items from §23.2 with one-line status"
#   bash scripts/ai_distill.sh examples/sentai_runtime/agent/agent.md \
#       "where does the camera ISR live and how is its priority pinned?"
#
# Knobs (env):
#   OLLAMA_HOST   default http://hpc.lan:11434
#   OLLAMA_MODEL  default qwen3-coder:30b
#   OLLAMA_CTX    default 16384  (token context window)
#   OLLAMA_MAX    default 600    (max tokens in response)

set -e
# Default model: qwen3.5:35b-a3b — MoE with 3B active params.
# Quality of a 35B model at the speed of a 3B model.  Optimal for
# summarization / search / extraction tasks (NOT code generation;
# leave that to Claude).  Override with OLLAMA_MODEL=qwen3:4b for
# trivial cases or qwen3-coder:30b for code-aware distillation.
HOST="${OLLAMA_HOST:-http://hpc.lan:11434}"
MODEL="${OLLAMA_MODEL:-qwen3.5:35b-a3b}"
CTX="${OLLAMA_CTX:-32768}"
MAX="${OLLAMA_MAX:-600}"

if [ "$1" = "--raw" ]; then
    PROMPT="$2"
elif [ $# -ge 2 ]; then
    FILE="$1"
    QUESTION="$2"
    if [ ! -f "$FILE" ]; then
        echo "ai_distill: file not found: $FILE" >&2
        exit 2
    fi
    # Guard: qwen3.5:35b-a3b is loaded with ctx_length=32768.  Inputs
    # beyond ~25k tokens (~100 KB / ~2000 lines) get SILENTLY TRUNCATED
    # and the model hallucinates in the missing tail.  Reject early
    # with a clear message — grep/sed the section first, then distill.
    FILE_BYTES=$(wc -c < "$FILE")
    if [ "$FILE_BYTES" -gt 100000 ]; then
        echo "ai_distill: file '$FILE' is $FILE_BYTES bytes (>100KB)." >&2
        echo "  qwen3.5:35b-a3b ctx is 32K tokens; >25K silently truncates." >&2
        echo "  Extract the section first (grep / sed -n 'A,Bp') and pipe via --raw." >&2
        exit 3
    fi
    PROMPT=$(printf '%s\n\n--- BEGIN FILE: %s ---\n%s\n--- END FILE ---\n' \
        "$QUESTION" "$FILE" "$(cat "$FILE")")
else
    echo "Usage:" >&2
    echo "  $0 <file> '<question>'" >&2
    echo "  $0 --raw '<full prompt>'" >&2
    exit 2
fi

# Encode prompt safely as JSON via python (avoids shell-quoting hell).
export MODEL MAX CTX
# think=False: for summarization/extraction we want direct output, not
# reasoning chains.  qwen3.x family default-on thinking eats all of
# num_predict in the `thinking` field and returns empty `response`.
JSON=$(python3 -c "
import json, sys, os
print(json.dumps({
    'model':  os.environ['MODEL'],
    'prompt': sys.stdin.read(),
    'stream': False,
    'think':  False,
    'options': {
        'temperature': 0.0,
        'num_predict': int(os.environ['MAX']),
        'num_ctx':     int(os.environ['CTX']),
    },
}))
" <<<"$PROMPT")

# Use temp file + curl -d @- so we don't hit ARG_MAX with large docs.
TMP=$(mktemp /tmp/ai_distill.XXXXXX.json)
trap 'rm -f "$TMP"' EXIT
printf '%s' "$JSON" > "$TMP"
curl -s --max-time 300 "$HOST/api/generate" -d @"$TMP" | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception as e:
    sys.stderr.write(f'ai_distill: bad response: {e}\n')
    sys.exit(3)
resp  = d.get('response', '').strip()
n_in  = d.get('prompt_eval_count', 0)
n_out = d.get('eval_count', 0)
t_eval_ms = d.get('eval_duration', 0) / 1e6
print(resp)
sys.stderr.write(f'[ai_distill] in={n_in} out={n_out} '
                  f'eval={t_eval_ms:.0f}ms '
                  f'tok/s={n_out / max(t_eval_ms/1000, 0.01):.1f}\n')
"
