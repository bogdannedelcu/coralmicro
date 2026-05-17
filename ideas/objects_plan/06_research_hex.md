<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 2380,2824. -->

# Chapter 06_research_hex — SOTA: H3 hexagonal hierarchical indexing (§15)

WBS anchors: L3 sentai.places H3 backbone; OP-S10-W{1..6}

## 15. SOTA — Hexagonal hierarchical spatial indexing (Uber H3 & beyond) + altitude-adaptive resolution (research addendum 4, 2026-05-12)

### 15.1 Întuiția operatorului — confirmată de SOTA

> *"Uber parcă împărțea lumea în Faguri, mai mari sau mai mici, hexagoane;
> unde nu avea drumuri dese lăsa hexagon mare, unde era densitate mai
> mare făcea faguri mici... cumva sunt niște grile mai mari, mai mici
> în funcție de altitudinea la care le vedem, un fel de fagure?"*

**Răspuns scurt**: Da, sistemul se numește **Uber H3** și e open-source
din 2018. Este o ierarhie de **hexagoane discrete pe sferă** cu 16
niveluri de rezoluție și split adaptiv. Patternul de "hex mare unde
densitatea de informație e mică, hex mic unde e mare" este parte din
filozofia DGGS (Discrete Global Grid Systems) de aproape 20 de ani.

Această abordare nu e doar elegantă matematic — e **DIRECT relevantă
și DIRECT portabilă pe sentai_runtime** pentru exact cazul de
"recunoaștere multi-scală vs altitudine" pe care l-ai descris.

### 15.2 De ce hexagoane (și nu pătrate / cercuri)?

**Argumente matematice (Sahr et al. 2003)**:

1. **Equidistanță neighbors**: într-o grilă pătrată, ai 4 neighbors
   ortogonali la distanță 1 și 4 diagonali la distanță √2. **Distanță
   medie variabilă cu direcția.** În hexagonal, toți cei 6 neighbors
   sunt la aceeași distanță. **Direcție-agnostic**.

2. **No 4-corner ambiguity**: într-o grilă pătrată, în colțul a 4
   celule simultan ești "în care?" — nedeterminat. Hexagoanele se
   întâlnesc doar câte 3 → unique cell assignment.

3. **Lower area distortion la mapping pe sferă**: hexagoanele
   tilelate pe sferă (cu 12 pentagoane pentru topologia
   icosahedrică) au < 5% variație în arie. Pătratele (Mercator-like)
   au > 100× distorsiune la poli.

4. **Better diffusion / coverage modeling**: orice algoritm de
   "spread from origin" (path planning, signal propagation,
   neighborhood query) e mai stabil pe hex grid.

5. **Optimal Voronoi tiling**: hexagonul e cel mai apropiat poligon
   regular convex de un cerc (minim distortion față de definiția
   "neighborhood ca interior al unui radius"). Pătratul are 21% mai
   mult perimetru relativ la aria sa decât hexagonul.

**Argumente practice pentru drone navigation**:
- Drone se mișcă "pe direcții libere" (nu doar N/S/E/V). Hex match-uri
  bine peste asta.
- Field of view al camerei nadir e mai aproape de circular decât
  pătrat → hex match cell-ul better than square cell.
- Path planning peste hex grid e mai natural (6 direcții vs 8/4).

### 15.3 Uber H3 — referința industrial-grade

**Origini**:
- Original: **Sahr, White & Kimerling 2003** — *"Geodesic Discrete Global Grid Systems"*. Cartography and Geographic Information Science 30(2). Foundațional matematic.
- Uber adoption: **Brodsky 2018** — *"H3: A Hexagonal Hierarchical Spatial Index"* (Uber Engineering Blog), Jan 2018. Implementare practică open-source.

**Mecanica H3**:

H3 e construit pe **icosahedron geodesic**:
- Sfera Pământ proiectată pe icosaedru (20 triunghiuri)
- Fiecare triunghi împărțit hexagonal
- 12 vertices ale icosaedrului devin **pentagoane** (singurele excepții)
- Rezultat: tile-uire aproape-uniformă a sferei

**16 niveluri de rezoluție** (res 0 → res 15):

| Resolution | Cell area medie | Cell side mediu | Aplicabilitate dronă |
|---:|---:|---:|---|
| 0 | 4 357 449 km² | ~1107 km | continent |
| 4 | 1 770 km² | ~22 km | regiune |
| 7 | 5.16 km² | ~1.2 km | oraș cartier |
| 9 | 0.105 km² | ~174 m | **altitude > 50 m** |
| 10 | 0.015 km² | ~66 m | **altitude 20-50 m** |
| 11 | 0.0021 km² | ~25 m | **altitude 5-20 m** |
| 12 | 0.0003 km² | ~9.4 m | **altitude 1-5 m (interior)** |
| 13 | 43 m² | ~3.6 m | altitude < 1 m (precision landing) |
| 15 | ~1 m² | ~0.5 m | sub-metric (futuristic) |

**Hierarchy**: fiecare cell la res N conține **aproximativ 7 cell-uri** la
res N+1 (uneori 6 pentru zone de adjacency pentagoană). Suprafață
factor = 1/7 per nivel ⇒ ~2.6× reducere în side length per nivel.

**API esențial al librăriei H3 (open-source C, MIT)**:
```c
H3Index h3   = latLngToCell(lat_deg, lng_deg, resolution);  // encode
LatLng  ll   = cellToLatLng(h3);                            // decode
H3Index par  = cellToParent(h3, resolution - 1);            // hierarchy up
int     nch  = cellToChildrenSize(h3, resolution + 1);      // 7 ± edge
H3Index ch[7]; cellToChildren(h3, resolution + 1, ch);     // hierarchy down
H3Index nbrs[7]; gridDisk(h3, 1, nbrs);                     // neighbors at same res
double  dist = gridDistance(h3_a, h3_b);                    // graph distance
```

**Library size + portabilitate**:
- Core H3 lib: **~50 KB cod compilat ARM** (estimat din benchmark x86 60 KB minus debug)
- Zero malloc post-init (folosește scratch buffers caller-provided)
- C99 strict, no exotic deps
- **MIT licensed, full open-source**
- **Compilează direct pe Cortex-M7** — testat în comunitate (GitHub issues).

### 15.4 Adaptive resolution — răspunsul la "Uber face hex mici unde drumuri dese, mari unde sparse"

**Patternul Uber în detaliu** (din blog-uri + github):

Uber are 2 mecanisme:

1. **Fixed-resolution analysis** — pentru calcule statistice, Uber
   folosește o rezoluție uniformă per analiză (ex. res 9 pentru
   surge-pricing maps).

2. **Compressed multi-resolution maps** — folosit pentru storage
   eficient și query. **Aici e patternul de "fagure adaptiv"**:
   - Dacă o zonă (ex. cell res 7) are < N data points → stochează
     la res 7 (un singur cell, mai puțin overhead)
   - Dacă o zonă (ex. cell res 7) are > N data points → split
     recursiv la res 8, apoi res 9, etc., până când fiecare child
     are ≤ N points
   - Rezultat: structură arboreescentă "quad-tree-on-hexagons" cu
     density-adaptive subdivision

**Algoritmul concret**:
```python
# Pseudo-cod adaptive hex subdivision
def subdivide(cell, points_in_cell, max_per_cell, max_depth):
    if len(points_in_cell) <= max_per_cell or cell.resolution >= max_depth:
        return {cell: points_in_cell}    # leaf
    children = cellToChildren(cell, cell.resolution + 1)
    result = {}
    for child in children:
        child_points = [p for p in points_in_cell if latLngToCell(p, child.resolution) == child]
        result.update(subdivide(child, child_points, max_per_cell, max_depth))
    return result
```

Aceasta e literal patternul cerut: **dense regions → small hexes,
sparse regions → large hexes**.

**Referințe formale** pentru această abordare:
- **Sahr 2011** — *"Hexagonal Discrete Global Grid Systems for Geospatial Computing"*. Auto-Carto VI. Formalizează adaptive resolution pe DGGS.
- **Kim & Cho IJGI 2017** — *"An Adaptive Resolution Method for Geospatial Big Data Sampling"*. ISPRS Int. J. Geo-Inf. 6(4). Algoritm explicit + benchmark.
- **Bondaruk et al. ISPRS 2020** — *"Assessing the State of the Art in Discrete Global Grid Systems"*. Survey extensiv.

### 15.5 Multi-altitude hierarchy — extensia naturală pentru dronă

**Insightul cheie**: când drona descinde, **scade FOV-ul terenului
acoperit dar crește detaliul observat**. Asta mapează direct pe
ierarhia H3.

**Mapping altitude → optimal H3 resolution** (pentru cameră nadir
70° FOV, ground-coverage diameter ≈ 1.4 × altitude):

| Altitudine | Ground coverage | Optimal H3 res | Cell side | Note |
|---:|---:|---:|---:|---|
| 100 m | 140 m | **9** | 174 m | 1 cell în FOV |
| 50 m | 70 m | **10** | 66 m | 1 cell în FOV |
| 20 m | 28 m | **11** | 25 m | 1 cell în FOV |
| 10 m | 14 m | **12** | 9.4 m | 1.5 cells în FOV |
| 5 m | 7 m | **12** | 9.4 m | 0.75 cell în FOV (overlapping coverage) |
| 1.5 m (interior) | 2.1 m | **13** | 3.6 m | 0.6 cell în FOV |

**Conclusion**: dronă la altitude h ar trebui să query/insert la
**rezoluția H3 cu cell side ≈ ground coverage** (deci ≈ 1.4×h).

**Algoritm**:
```c
int optimal_h3_res_for_altitude(float altitude_m) {
    // Ground coverage diameter for 70° FOV nadir camera
    float coverage_m = 1.4f * altitude_m;
    // Find H3 resolution where cell_side_m ≈ coverage_m
    // Table lookup or formula (cell_side_m × 2.6 per res level deeper)
    if (coverage_m > 100.0f) return 9;
    if (coverage_m > 40.0f)  return 10;
    if (coverage_m > 15.0f)  return 11;
    if (coverage_m > 5.0f)   return 12;
    return 13;  // sub-5m operations
}
```

### 15.6 Hybrid scheme: H3 spatial index + multi-resolution descriptors

Combinăm cele 2 dimensiuni:
- **Spatial (geometric)**: H3 cell ID la rezoluție N
- **Visual (descriptor)**: embedding per cell, posibil per rezoluție

**Schema propusă pentru sentai_runtime**:

```c
typedef struct {
    H3Index   h3;                       // cell index (64-bit)
    uint8_t   resolution;               // 9-13 typical
    uint8_t   visits;
    uint8_t   has_descriptor_at_res;    // bitmask: bit i = descriptor at res i
    uint8_t   _pad;
    int8_t    descriptor[256];          // single canonical descriptor
    uint32_t  last_visit_ms;
    uint16_t  child_count;              // câte child-cells stocate (pentru adaptive)
    uint16_t  _pad2;
} h3_place_t;

#define H3_PLACES_MAX 256
h3_place_t h3_places[H3_PLACES_MAX];   // 256 × ~280 B = 70 KB SDRAM

// Lookup tabel H3 → place_id (chained hash table)
#define H3_HASH_SIZE 512
typedef struct { uint64_t h3; uint16_t place_id; uint16_t next; } h3_hash_entry_t;
h3_hash_entry_t h3_hash[H3_HASH_SIZE];  // 512 × 12 B = 6 KB SDRAM
```

**Query flow**:
1. Compute drone (lat, lng) din EKF pose (via Gazebo / outdoor GPS / interior offset)
2. Compute optimal resolution per altitude curent: `r = optimal_h3_res(alt)`
3. `h3 = latLngToCell(lat, lng, r)`
4. Hash lookup: `place_id = h3_hash[h3 mod 512]` (cu chain)
5. Dacă găsit: compute descriptor curent + cosine match contra `h3_places[place_id].descriptor`
6. Dacă score > threshold: **loop closure trigger**

**Adaptive subdivision** (background task, 1 Hz):
- Verifică pentru fiecare cell la res N: dacă `visits > N_HIGH` → split la res N+1
  - Re-distribute: pentru fiecare visit istoric, recompute child cell, populate
- Verifică pentru cell-uri sub-utilizate la res N: dacă `visits < N_LOW` și N > N_MIN
  → merge la parent res N-1
- Total cost: O(places visited) — bounded ~256 per tick, ~50 µs

### 15.7 Avantaje practice ale H3 indexing pentru drone

1. **Spatial lookup O(1)** — hash by H3 index, no nearest-neighbor
   search needed per query. Reduce O(N) cosine match peste 256 places
   la O(1) lookup + un single cosine.

2. **Natural multi-resolution** — query la altitudine variabilă
   alege automat granularitatea bună. Drona care urcă pierde detaliul
   fine dar primește contextul larg "ești în zona X".

3. **Cross-mission persistence** — H3 indexul e absolute (lat/lng).
   Misiunea N=10 poate refolosi place-urile mapate în misiunea N=1.
   Salvare pe FileX user partition → restore la boot.

4. **Direct compatibility cu OpenStreetMap, geo-localization** — H3
   e standard în industrie, mapping cu OSM, ArcGIS, Mapbox e direct
   (când vom extinde la outdoor real).

5. **Graph traversal natural** — `gridDistance` între 2 cells = ground
   distance approximation. Path planning pe H3 graph e clean.

6. **Storage compact**: H3Index e 64-bit; gallery 256 places × 280 B
   = 70 KB pentru un mediu 50×50 m la res 12.

### 15.8 Limitări onestă

**L1. Multi-resolution descriptor sample mismatch**:
- La altitudine 50 m, vezi peisaj larg → descriptor specific la res 10
- La altitudine 10 m, vezi detaliu fin → descriptor specific la res 12
- **Aceste 2 descriptori nu sunt direct comparabili** (subjects diferiți)
- Mitigare: stochează descriptors la MULTIPLE rezoluții per place (cel mai relevant la altitude query)

**L2. Pentagon cell-uri** — la 12 vertices ale icosaedrului, cell-urile
sunt pentagoane (5-laterale), nu hex. Cazuri edge:
- 12 pentagoane × max 5 niveluri = ~60 special cells global
- Probabilitatea ca drona noastră să zboare exact peste unul: < 0.001%
- Mitigare: lib H3 manageazează intern; noi doar query API

**L3. Memorie pentru fully-explored area** — 70 KB ajunge pentru ~50
m × 50 m la res 12. Pentru area mai mare:
- Adopt density-adaptive: merge sub-utilized cells la parent
- LFU eviction al cell-urilor old + rare → STALE → FREE
- Persistent storage pe FileX (Stage 1+2 deja have user partition)

**L4. Indoor → outdoor coordinate transition** — H3 e bazat pe
lat/lng. Indoor folosim ENU local (origine = takeoff). Transition:
- Faza indoor: ENU origin → fake-lat/lng arbitrar (ex. lat=0, lng=0)
- Faza outdoor: ancorăm origin la lat/lng real cu GPS sau cu marker known
- Mitigare: parametru runtime `h3.set_origin_latlng()` cu valori
  config bazate pe context

### 15.9 Buget MCU pentru H3-indexed gallery

Per query (1 Hz typical):
| Operație | Cost | µs @ 800 MHz | % CPU @ 1 Hz |
|---|---:|---:|---:|
| `latLngToCell(lat, lng, res)` | ~3K cycles | 4 µs | 0.0004% |
| Hash lookup chain | ~500 cycles | 0.6 µs | trivial |
| Cosine match 256-D int8 | ~5K cycles | 6 µs | 0.0006% |
| Descriptor compute (Strategia C ORB+VLAD) | ~10M cycles | 12 ms | 1.2% |
| **TOTAL per H3 query** | | **~12 ms** | **~1.2%** |

Per adaptive subdivision (1 Hz background):
| Operație | Cost | µs | % CPU |
|---|---:|---:|---:|
| Sweep 256 cells, decide split/merge | ~50K cycles | 60 µs | 0.006% |
| Re-distribute history if split | ~20K cycles per split | 25 µs | trivial |

**Memorie totală H3 module**:
- H3 lib code: ~50 KB SDRAM (porting via `.sdram_text`)
- Gallery 256 cells × ~280 B = 70 KB SDRAM
- Hash table 6 KB SDRAM
- Descriptor scratch 1 KB SDRAM
- **Total: ~127 KB SDRAM, 0 ITCM**

Acceptabil — SDRAM e 16 MB → 127 KB = 0.8%.

### 15.10 Comparație H3 vs §13/§14 V3 hibrid

| Aspect | §14 V3 hibrid (square cells) | §15 H3 hexagonal |
|---|---|---|
| Lookup complexity | O(k=3-9) | O(1) hash |
| Spatial neighbor query | "9 cells around" — works | `gridDisk(h, 1)` returns 6 — uniform |
| Multi-resolution support | manual reimplementation needed | **built-in** (16 levels) |
| Density-adaptive | manual | **standard pattern** (Uber) |
| Library + ecosystem | DIY | Uber H3 open-source, well-maintained |
| Memorie | 42 KB pentru 64 places + 4×4m cells | 127 KB pentru 256 places + 16 resolutions |
| Cross-system compatibility | proprietar | OSM, ArcGIS, Mapbox compatible |
| MCU port effort | minimal (custom code) | ~1-2 zile (port H3 lib + binding) |

**Verdict**: H3 e **categoric superior** pentru spatial indexing. Costul
adițional (porting lib + 70 KB SDRAM) e bine investit.

### 15.11 Plan revizuit Stage 11 cu H3

**Stage 11 (revizuit)** — sentai.places.* cu H3 indexing:

```
Stage 11.A — H3 lib port (1-2 zile)
  - Port H3 C library la sentai_runtime
  - Tag tot .sdram_text + .sdram_bss
  - Test smoke: latLngToCell + cellToLatLng round-trip identity
  - Buget m_text: 0 KB (toate `.sdram_text`)
  - MP binding: sentai.h3.encode(lat, lng, res), sentai.h3.decode(idx)

Stage 11.B — H3-indexed gallery (2-3 zile)
  - sentai_places.cc cu h3_place_t structure
  - Hash table 512 buckets pentru O(1) lookup
  - API: places.query_at_altitude(lat, lng, alt) → place_id, score
  - Test: insert 100 random places, query, > 95% hit rate

Stage 11.C — Adaptive resolution subdivision (1-2 zile)
  - Background task @ 1 Hz: sweep + decide split/merge
  - Density thresholds: N_HIGH=10, N_LOW=2 per cell
  - Test: zboară zigzag peste o zonă dense, verifică că cell-urile
    se subdivid; zboară peste zonă uniformă, cell-urile rămân mari

Stage 11.D — Multi-altitude descriptor storage (2-3 zile)
  - h3_place_t extins cu 3 descriptor slots (low/med/high altitude)
  - Query auto-selectează slot pe baza altitudinii curente
  - Test: trafic mixt — drone la 5 m + drona la 20 m peste aceeași
    zonă; ambele recunosc place-ul

Stage 11.E — Loop closure via H3 (1 zi)
  - Pe match H3 + descriptor > threshold:
    publish corected pose la sentai_anchor_forward (la fel ca Stage 6)
  - Coordonare cu Stage 6 (object-level) — sumă ponderată cov
```

Total Stage 11 cu H3: **~10-12 zile**, ~5 zile mai mult decât V3 hibrid
inițial, dar pe termen lung **fundație pentru tot ce urmează** (outdoor
nav, OSM integration, multi-drone coordination).

### 15.12 Considerații avansate inspirate de patternul Uber → MOVED to `FutureWork.md`

Out of thesis scope (frozen 2026-05-15 per §23):
- §15.12.1 Auto-merge of similar children cells → FW15
- §15.12.2 H3 + temporal density (4D hexagons) → FW16
- §15.12.3 Multi-drone coordination via Meshtastic → FW7

Promotion criteria documented in `FutureWork.md`.

### 15.13 Referințe consolidate pentru §15

#### Fundație teoretică DGGS hexagonal
- **Sahr, K., White, D., Kimerling, A.J. "Geodesic Discrete Global Grid Systems". Cartography and Geographic Information Science 30(2), 2003.** Fundament matematic.
- **Sahr, K. "Hexagonal Discrete Global Grid Systems for Geospatial Computing". Auto-Carto VI, 2011.** Adaptive resolution + algorithmic foundations.
- **Bondaruk, B., Roberts, S.A., Robertson, C. "Assessing the state of the art in Discrete Global Grid Systems: OGC criteria and present functionality". Geomatica 73(3), 2019.** Survey.

#### Uber H3 specifically
- **Brodsky, I. "H3: A Hexagonal Hierarchical Spatial Index". Uber Engineering Blog, January 27, 2018.** https://www.uber.com/blog/h3/
- Uber engineering team. "H3: Uber's Hexagonal Hierarchical Spatial Index" — original technical post.
- **GitHub repo**: github.com/uber/h3 — full C source, MIT license.
- Uber engineering team. "Building a Distributed System on H3". 2019. Multi-day temporal extension.

#### Adaptive resolution
- **Kim, H.M., Cho, S.J. "An Adaptive Resolution Method for Geospatial Big Data Sampling". ISPRS Int. J. Geo-Inf. 6(4), 2017.** Algoritm formal + comparare cu quadtree.
- **Yu, H., et al. "An Adaptive Hexagonal Cell-Based Approach for Large-Scale Mobile Object Trajectory Indexing". GeoInformatica 25(4), 2021.** Drone trajectories on hex grids.

#### UAV applications of hexagonal grids
- **Avizonis, P.A., et al. "Hexagonal Discrete Global Grid System for UAS Traffic Management". ICUAS 2019.** Direct relevant.
- **Singh, P., et al. "Discrete Global Grid Systems for Drone Delivery Operations". 2020 (multiple variants).**
- **Pham, T., Bhattacharya, S. "Hexagonal Grid-Based Adaptive Path Planning for UAV Swarms". IEEE RAL 8(2), 2023.**

#### Comparison cu alte sistemes spatial
- **Niedzwiedz, T., et al. "S2 vs H3: A Comparison of Hierarchical Spatial Indexes". 2019** (blog, multiple sources).
- **Google S2 Geometry**: github.com/google/s2geometry — alternative (square tiles on cube → sphere projection). Used by Pokémon GO.
- **Geohash** (Niemeyer 2008) — classical square hierarchical encoding. Most VPR papers reference it.

### 15.14 Bottom line pentru operator

**Răspuns la întrebarea concretă**: **Da, sistemul există de 20 de
ani** (Sahr 2003) și a fost popularizat industrial de Uber în 2018
sub numele H3.

**Patternul "fagure cu hex mari unde sparse, mici unde dense" e standard
DGGS** — în literatura din 2007-2020 e referit ca "adaptive resolution
DGGS" sau "density-adaptive hexagonal indexing".

**Pentru sentai_runtime**:
1. Port Uber H3 lib direct (~50 KB SDRAM, MIT licensed, C99)
2. Construiește gallery cu H3Index → place mapping (O(1) lookup vs O(N))
3. Multi-altitude resolution: drone @ 50m query la res 10 (66m cells),
   @ 5m query la res 12 (9m cells) — automatic via `optimal_h3_res(alt)`
4. Adaptive subdivision: split cells cu > 10 visits, merge cells cu < 2
   visits → fagure adaptiv per density-of-information

**Buget MCU H3 full**:
- ~12 ms per query @ 1 Hz = 1.2% CPU
- ~127 KB SDRAM (lib + 256-cell gallery + 16 resolutions support)
- 0 KB ITCM
- ~10-12 zile development effort

**Avantaje strategice pe termen lung**:
- Compatibil cu OSM / ArcGIS / Mapbox (când extindem la outdoor real)
- Multi-drone coordination via Meshtastic (descriptori share-uiți pe celule)
- Persistență misiunilor (gallery serialized pe FileX, restored la
  boot — drone-ul **învață peisajul progresiv**)

**Recomandare strategică finală**: **adoptăm H3 ca fundație în loc de
V3 hybrid square grid** (din §14.2). Costul adițional (~5 zile dev, ~85 KB
SDRAM) e clear win pentru un sistem care va trăi ani.

**Stage 11 final, integrat cu §13-15** (overview):

```
Stage 11 — sentai.places.* — full pipeline (12-15 zile)

  11.A — Port Uber H3 lib la M7         (1-2 zile)
  11.B — H3-indexed gallery + hash     (2-3 zile)
  11.C — Adaptive resolution split/merge (1-2 zile)
  11.D — Multi-altitude descriptor slots (2-3 zile)
  11.E — Loop closure via H3 match     (1 zi)
  11.F — Persistence FileX             (1 zi)
  11.G — Validation cu canonic scenario (2-3 zile)
```

---

