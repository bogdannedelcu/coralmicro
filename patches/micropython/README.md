# SentAI patches for the `micropython` submodule

Apply on top of the upstream MicroPython v1.23.0 pinned by
`third_party/micropython`.  Applied automatically at configure time by
[`scripts/apply_sdk_patches.sh`](../../scripts/apply_sdk_patches.sh)
(same mechanism as `patches/coralmicro-rt1176-sdk/`).

The ARM build compiles the embed-package copy under
`examples/sentai_runtime/micropython_embed/py/` (tracked in git).  That
copy must carry the same change; `/qstr-regen` regenerates it from the
(patched) submodule, so keep both in sync.

## Patch index

### `0001-objint-longlong-from-str-len-honour-len.patch`

`py/objint_longlong.c: mp_obj_new_int_from_str_len()` (used when
`MICROPY_LONGINT_IMPL == LONGLONG`, which SentAI selects) parsed the
literal with `strtoll(*str, ...)` and ignored `len`, assuming the string
is NUL-terminated.  The parser hands it `lex->vstr.buf`, which is **not**
terminated: `strtoll` kept reading the stale tail left in the vstr by a
longer previous token.  Any literal that overflows the 31-bit small int
(e.g. `0x40C04008`, `1086341128`) therefore failed with
`SyntaxError: invalid syntax for integer with base N` whenever an earlier
token on the same line (typically a format string) was longer than the
literal — found while bisecting REPL pastes in EXP-s248 (2026-10-31).

Fix: bounded digit loop honouring `len`, `_` separators accepted,
explicit `OverflowError` instead of silent wrap, and the caller's `neg`
flag applied (strtoll never saw the sign — `int("-1086341128")` returned
a positive value).

Upstream status: the two `TODO`s are still present in MicroPython master;
the unix/stm32 ports use the MPZ long-int implementation and never hit
this path.
