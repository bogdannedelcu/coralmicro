#!/bin/bash
# s119 — build & run H3 smoke test on x86 Linux.
# Compiles third_party/h3/src/h3lib/lib/*.c into a static lib, links our
# smoke test against it, and runs it.
set -e

ROOT="$(cd "$(dirname "$0")"/../../../.. && pwd)"
H3_ROOT="$ROOT/third_party/h3"
BUILD_DIR="$(dirname "$0")/build"
mkdir -p "$BUILD_DIR"

# 1. Generate h3api.h from h3api.h.in (version macros substitution only)
H3_VERSION_MAJOR=4
H3_VERSION_MINOR=4
H3_VERSION_PATCH=1
H3API_IN="$H3_ROOT/src/h3lib/include/h3api.h.in"
H3API_OUT="$BUILD_DIR/h3api.h"
sed -e "s/@H3_VERSION_MAJOR@/$H3_VERSION_MAJOR/g" \
    -e "s/@H3_VERSION_MINOR@/$H3_VERSION_MINOR/g" \
    -e "s/@H3_VERSION_PATCH@/$H3_VERSION_PATCH/g" \
    "$H3API_IN" > "$H3API_OUT"
echo "[s119] generated $H3API_OUT ($(wc -l < $H3API_OUT) lines)"

# 2. Compile lib into static archive
H3_LIB_DIR="$H3_ROOT/src/h3lib/lib"
H3_INC_DIR="$H3_ROOT/src/h3lib/include"
OBJS=""
for src in "$H3_LIB_DIR"/*.c; do
    obj="$BUILD_DIR/$(basename ${src%.c}).o"
    gcc -O2 -c -I"$H3_INC_DIR" -I"$BUILD_DIR" "$src" -o "$obj"
    OBJS="$OBJS $obj"
done
ar rcs "$BUILD_DIR/libh3.a" $OBJS
echo "[s119] built libh3.a ($(stat -c %s $BUILD_DIR/libh3.a) bytes)"

# 3. Compile + link smoke test
gcc -O2 -I"$H3_INC_DIR" -I"$BUILD_DIR" \
    "$(dirname "$0")/h3_smoke.c" \
    "$BUILD_DIR/libh3.a" \
    -lm -o "$BUILD_DIR/h3_smoke"
echo "[s119] built smoke binary"

# 4. Run
echo "[s119] ---"
"$BUILD_DIR/h3_smoke"
RC=$?
echo "[s119] --- exit=$RC ---"
exit $RC
