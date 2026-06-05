---
name: ai-distill-tool
description: "scripts/ai_distill.sh shells out to local Ollama on hpc.lan (qwen3.5:35b-a3b MoE) for file summarization / targeted extraction. Use this BEFORE reading large files into agent context. Hard 100 KB input cap — beyond that the model silently truncates + hallucinates. Returns ~40 tok/s, sub-10s wallclock for typical distill tasks."
metadata: 
  node_type: memory
  type: reference
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Wrapper**: `scripts/ai_distill.sh <file> "<question>"` (also `--raw "<prompt>"`).

**Endpoint**: `http://hpc.lan:11434/api/generate` (RTX 4090 box on LAN).
Allowed in `.claude/settings.local.json` for `curl http://hpc.lan:11434/*`.

**Default model**: `qwen3.5:35b-a3b` — Qwen3.5 MoE, 36B total params,
3B active per token.  Q4_K_M, ~23 GB VRAM.  Loaded with ctx_length=32768.

**Use cases (validated)**:
- Summarize a .md doc to 60-100 words (TASK 1 in benchmark, perfect)
- Extract specific values from a source file (TASK 2, perfect — pulled
  takeoff height, closure tolerance, fn name correctly)

**Hard limit**: input > 100 KB triggers a hard refuse (exit 3) with a
"grep / sed the section first" message.  Larger inputs silently
truncate at 32K tokens and the model HALLUCINATES in the missing
tail (TASK 3 in benchmark: invented "STM32H7", "Two-Flight protocol",
§19 fictiv when fed 160 KB of objects_plan.md).  Hard rule: never
disable this guard without a smaller-context model.

**Workflow for large files**:
```bash
sed -n '4582,4691p' ideas/objects_plan.md > /tmp/section.md
bash scripts/ai_distill.sh /tmp/section.md "extract the 8 row table…"
```

**Disabling reasoning**: wrapper passes `"think": false` at top-level
because qwen3.x family defaults to thinking-mode which eats the entire
`num_predict` budget in the `thinking` field and returns empty `response`.

**Speed**: 38-40 tok/s output generation, 5-10s wallclock for typical
short-input tasks (1-3 KB input).  Below theoretical max for the
arch but adequate.

**NOT for**: code generation, architectural decisions, security review,
debugging — those stay with Claude.  Distill is for "filter / extract /
summarize", nothing load-bearing.

**NOT for full-document translation** (verified 2026-05-17):
operator flagged hallucination risk on load-bearing plan docs.  Test
on `ideas/objects_plan/10_bibliography.md` (120 lines) at
`OLLAMA_MAX=8000` produced:
- 3 missing content lines (117 out of 120)
- "--- BEGIN FILE: <path> ---" preamble artifact in output (model
  echoed the wrapper's delimiters as part of its response)
- Subtle technical drift risk on citation names / dates / numbers
The 3B-active-param MoE is not reliable enough to translate planning
docs that drive the thesis.  Operator decision: "mai bine nu folosim,
riscam să introducem halucinări chiar în mijlocul planului nostru".
Translations of `ideas/objects_plan/*.md` and `ideas/objects.md` stay
on the translate-on-pass human-driven path per [[english-docs-only]].

Related: [[ai-distill-tool]] is documented in CLAUDE.md so future
sessions discover the tool without needing memory lookup.
