/*
 * MicroPython config for the SentAI x86/Linux SIM build.
 *
 * Includes the FIRMWARE mpconfigport.h verbatim so the QSTR table stays
 * 1:1 with the ARM build (otherwise the precomputed
 * micropython_embed/genhdr/qstrdefs.generated.h hashes don't match what
 * each .c file references → undefined-symbol cascade).
 *
 * The firmware mpconfigport declares two extern hooks
 * (mp_embed_enter_critical / mp_embed_exit_critical) that on ARM live in
 * mp_embed_safe.c with FreeRTOS taskENTER_CRITICAL.  On SIM we provide
 * stub implementations in main_sim.c — single REPL task in Phase 1, no
 * cross-task atomic-section pressure yet.  Phase 3+ (crazy bridge over
 * UART socket) will replace the stubs with real FreeRTOS critical
 * sections.
 */

#include "../examples/sentai_runtime/mpconfigport.h"
