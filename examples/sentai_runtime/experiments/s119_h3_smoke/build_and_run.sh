#!/usr/bin/env bash
# s119 — H3 smoke test (ObjectsPlan L1).
# Standalone, x86 host. Builds H3 v4.4.1 + h3_smoke.c, runs the binary,
# exits non-zero if any assertion fails.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
H3="$HERE/../../../../third_party/h3"
BUILD="$HERE/build"

mkdir -p "$BUILD"

# Generate h3api.h from h3api.h.in by substituting the four version macros.
# CMake usually does this with configure_file; here we do it with sed so we
# stay independent of the CMake superproject.
VERSION="$(cat "$H3/VERSION")"
MAJOR="${VERSION%%.*}"
REST="${VERSION#*.}"
MINOR="${REST%%.*}"
PATCH="${REST#*.}"

sed -e "s/@H3_VERSION_MAJOR@/${MAJOR}/g" \
    -e "s/@H3_VERSION_MINOR@/${MINOR}/g" \
    -e "s/@H3_VERSION_PATCH@/${PATCH}/g" \
    "$H3/src/h3lib/include/h3api.h.in" > "$BUILD/h3api.h"

CC="${CC:-gcc}"
CFLAGS="-O2 -Wall -Wextra -Wno-unused-parameter -std=c11 -D_DEFAULT_SOURCE"
CFLAGS="$CFLAGS -I$BUILD -I$H3/src/h3lib/include"

LIB_SOURCES=(
    "$H3/src/h3lib/lib/algos.c"
    "$H3/src/h3lib/lib/area.c"
    "$H3/src/h3lib/lib/baseCells.c"
    "$H3/src/h3lib/lib/bbox.c"
    "$H3/src/h3lib/lib/cellsToMultiPoly.c"
    "$H3/src/h3lib/lib/directedEdge.c"
    "$H3/src/h3lib/lib/faceijk.c"
    "$H3/src/h3lib/lib/h3Assert.c"
    "$H3/src/h3lib/lib/h3Index.c"
    "$H3/src/h3lib/lib/iterators.c"
    "$H3/src/h3lib/lib/latLng.c"
    "$H3/src/h3lib/lib/linkedGeo.c"
    "$H3/src/h3lib/lib/localij.c"
    "$H3/src/h3lib/lib/mathExtensions.c"
    "$H3/src/h3lib/lib/polyfill.c"
    "$H3/src/h3lib/lib/polygon.c"
    "$H3/src/h3lib/lib/vec2d.c"
    "$H3/src/h3lib/lib/vertex.c"
)

echo "[s119] compiling H3 v${VERSION} + smoke ..."
"$CC" $CFLAGS "${LIB_SOURCES[@]}" "$HERE/h3_smoke.c" -o "$BUILD/h3_smoke" -lm

echo "[s119] running smoke ..."
"$BUILD/h3_smoke"
