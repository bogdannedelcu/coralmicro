/* s119 — H3 submodule smoke test
 *
 * Compiles standalone against third_party/h3/src/h3lib/lib/*.c with a
 * hand-substituted h3api.h. Verifies the four API entry points sentai.places
 * needs: latLngToCell, cellToLatLng, gridDisk, cellToBoundary.
 *
 * No FreeRTOS, no MicroPython — pure libc + libm.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "h3api.h"

#define DEG2RAD(d) ((d) * (M_PI / 180.0))
#define RAD2DEG(r) ((r) * (180.0 / M_PI))

/* Shared by all tests: the cell computed in test_latlng_to_cell. */
static H3Index g_test_cell = 0;
static const double g_test_lat_deg = 40.689167;
static const double g_test_lng_deg = -74.044444;

static int test_latlng_to_cell(void) {
    LatLng p = { .lat = DEG2RAD(g_test_lat_deg), .lng = DEG2RAD(g_test_lng_deg) };
    H3Error err = latLngToCell(&p, 9, &g_test_cell);
    if (err) {
        printf("[h3] FAIL latLngToCell err=%d\n", err);
        return 1;
    }
    printf("[h3] latLngToCell(%.6f, %.6f, res=9) -> %llx\n",
           g_test_lat_deg, g_test_lng_deg, (unsigned long long)g_test_cell);
    /* Validity check: cell must be non-zero + isValidCell */
    if (g_test_cell == 0 || !isValidCell(g_test_cell)) {
        printf("[h3] FAIL cell %llx is not valid\n", (unsigned long long)g_test_cell);
        return 2;
    }
    return 0;
}

static int test_cell_to_latlng(void) {
    LatLng p;
    H3Error err = cellToLatLng(g_test_cell, &p);
    if (err) {
        printf("[h3] FAIL cellToLatLng err=%d\n", err);
        return 3;
    }
    double lat_deg = RAD2DEG(p.lat);
    double lng_deg = RAD2DEG(p.lng);
    printf("[h3] cellToLatLng(%llx) -> (%.6f, %.6f)\n",
           (unsigned long long)g_test_cell, lat_deg, lng_deg);
    /* res=9 cell edge ~0.174 km, so center is ≤0.001° from any point inside */
    if (fabs(lat_deg - g_test_lat_deg) > 0.005 ||
        fabs(lng_deg - g_test_lng_deg) > 0.005) {
        printf("[h3] FAIL roundtrip outside 0.005° tolerance\n");
        return 4;
    }
    return 0;
}

static int test_grid_disk(void) {
    H3Index origin = g_test_cell;
    int64_t n;
    H3Error err = maxGridDiskSize(1, &n);
    if (err || n != 7) {
        printf("[h3] FAIL maxGridDiskSize err=%d n=%lld\n", err, (long long)n);
        return 5;
    }
    H3Index out[7] = {0};
    err = gridDisk(origin, 1, out);
    if (err) {
        printf("[h3] FAIL gridDisk err=%d\n", err);
        return 6;
    }
    int populated = 0;
    for (int i = 0; i < 7; i++) if (out[i] != 0) populated++;
    printf("[h3] gridDisk k=1 -> %d cells (origin + 6 ring)\n", populated);
    if (populated != 7) {
        printf("[h3] FAIL gridDisk not full (got %d)\n", populated);
        return 7;
    }
    return 0;
}

static int test_boundary(void) {
    CellBoundary b;
    H3Error err = cellToBoundary(g_test_cell, &b);
    if (err) {
        printf("[h3] FAIL cellToBoundary err=%d\n", err);
        return 8;
    }
    printf("[h3] boundary -> %d vertices\n", b.numVerts);
    if (b.numVerts != 6) {
        printf("[h3] FAIL hex boundary should have 6 verts (got %d)\n", b.numVerts);
        return 9;
    }
    return 0;
}

int main(void) {
    int rc = 0;
    rc |= test_latlng_to_cell();
    rc |= test_cell_to_latlng();
    rc |= test_grid_disk();
    rc |= test_boundary();
    if (rc == 0) {
        printf("[h3] PASS\n");
        return 0;
    }
    printf("[h3] FAIL rc=%d\n", rc);
    return rc;
}
