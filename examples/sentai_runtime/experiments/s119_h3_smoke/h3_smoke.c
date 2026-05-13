// s119 — H3 smoke test (ObjectsPlan L1, written from scratch 2026-05-13).
//
// Purpose: prove that the freshly-vendored `third_party/h3` library
// compiles cleanly against the host SIM toolchain and that the four
// API entry points sentai.places (L3) will call return sane values.
//
// What we exercise (and the invariants we check):
//
//   latLngToCell      — coords -> H3 cell (round-trip lossiness bounded)
//   cellToLatLng      — cell  -> coords  (inverse must be within cell radius)
//   gridDisk          — k-ring neighbourhood (size = 3k(k+1)+1)
//   cellToBoundary    — hex polygon (6 vertices unless cell is a pentagon)
//
// Pass criterion: every assertion below is true and the program prints "PASS"
// at the end. Any failure prints "FAIL ..." and exits non-zero.

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "h3api.h"

static int fail = 0;

#define EXPECT(cond, msg)                                            \
    do {                                                             \
        if (!(cond)) {                                               \
            fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            fail = 1;                                                \
        }                                                            \
    } while (0)

// Haversine distance in metres between two (lat, lng) pairs given in radians.
static double haversine_m(LatLng a, LatLng b) {
    const double R = 6371000.8;
    double dlat = b.lat - a.lat;
    double dlng = b.lng - a.lng;
    double s = sin(dlat / 2.0);
    double c = sin(dlng / 2.0);
    double h = s * s + cos(a.lat) * cos(b.lat) * c * c;
    return 2.0 * R * asin(sqrt(h));
}

int main(void) {
    // Origin: Bucharest centre (drone testbed-relative city). Deliberately
    // chosen distinct from any documented example to make sure we exercise
    // the API and not a memorised demo.
    LatLng origin_deg = { .lat = 44.4268, .lng = 26.1025 };
    LatLng origin = {
        .lat = origin_deg.lat * M_PI / 180.0,
        .lng = origin_deg.lng * M_PI / 180.0,
    };

    // Resolution 8: cell average edge ~0.46 km, area ~0.74 km^2 -- coarse
    // enough that one cell covers a small park, fine enough to discriminate
    // city blocks. Good fit for sentai.places gallery binning.
    const int res = 8;

    // 1. latLngToCell
    H3Index cell = 0;
    H3Error e = latLngToCell(&origin, res, &cell);
    EXPECT(e == E_SUCCESS, "latLngToCell returned non-success");
    EXPECT(cell != 0, "latLngToCell produced zero index");
    printf("[h3] latLngToCell(%.6f, %.6f, res=%d) -> %" PRIx64 "\n",
           origin_deg.lat, origin_deg.lng, res, cell);

    // 2. cellToLatLng (inverse) must land inside the cell radius (~460 m).
    LatLng back = {0};
    e = cellToLatLng(cell, &back);
    EXPECT(e == E_SUCCESS, "cellToLatLng returned non-success");
    double drift_m = haversine_m(origin, back);
    printf("[h3] cellToLatLng -> (%.6f, %.6f) drift=%.1f m\n",
           back.lat * 180.0 / M_PI, back.lng * 180.0 / M_PI, drift_m);
    EXPECT(drift_m < 700.0, "round-trip drift exceeds res-8 cell radius");

    // 3. gridDisk k=1 -> 7 cells (origin + 6 neighbours).
    H3Index ring1[7] = {0};
    e = gridDisk(cell, 1, ring1);
    EXPECT(e == E_SUCCESS, "gridDisk k=1 returned non-success");
    int seen_origin = 0, nonzero = 0;
    for (int i = 0; i < 7; ++i) {
        if (ring1[i] != 0) ++nonzero;
        if (ring1[i] == cell) ++seen_origin;
    }
    printf("[h3] gridDisk k=1 -> %d non-zero, origin included %d time(s)\n",
           nonzero, seen_origin);
    EXPECT(nonzero == 7, "gridDisk k=1 should fill all 7 slots");
    EXPECT(seen_origin == 1, "gridDisk k=1 must include origin exactly once");

    // 3b. gridDisk k=2 -> 19 cells = 3*2*(2+1)+1.
    H3Index ring2[19] = {0};
    e = gridDisk(cell, 2, ring2);
    EXPECT(e == E_SUCCESS, "gridDisk k=2 returned non-success");
    nonzero = 0;
    for (int i = 0; i < 19; ++i) {
        if (ring2[i] != 0) ++nonzero;
    }
    EXPECT(nonzero == 19, "gridDisk k=2 should fill all 19 slots");

    // 4. cellToBoundary: 6 vertices for a hex (origin is not a pentagon
    // at res 8 in Bucharest).
    CellBoundary boundary = {0};
    e = cellToBoundary(cell, &boundary);
    EXPECT(e == E_SUCCESS, "cellToBoundary returned non-success");
    printf("[h3] cellToBoundary -> %d vertices\n", boundary.numVerts);
    EXPECT(boundary.numVerts == 6, "Bucharest res-8 cell should be hex");
    EXPECT(!isPentagon(cell), "origin must not be a pentagon for this test");

    if (fail) {
        fprintf(stderr, "s119 H3 smoke: FAIL\n");
        return 1;
    }
    printf("s119 H3 smoke: PASS\n");
    return 0;
}
