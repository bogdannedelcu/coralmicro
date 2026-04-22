# SentAI Optical-Flow → Altitude (AGL) Estimation

**Context**: Ambele camere OV5640 privesc **în jos** din dronă.
FOV-ul lensului (măsurat empiric pe modulul montat pe Coral Dev Board
Micro, confirmat de utilizator 2026-04-21):
- **Latura lungă a senzorului (640 px)**: **58°** FOV
- **Latura scurtă a senzorului (480 px)**: **45°** FOV
- Ambele camere aliniate pe latura lungă

## Geometria cheie

Pentru o cameră rectilineară cu FOV α și senzor de N pixeli pe axa
respectivă, **focal length în pixeli** este:

```
f_px = (N / 2) / tan(α / 2)
```

### Raw 640×480

```
f_px_raw_h = 320 / tan(29°)   = 320 / 0.5543 = 577.3 px
f_px_raw_v = 240 / tan(22.5°) = 240 / 0.4142 = 579.4 px
```

Ambele dau ~578 px — confirmă coerența lensului (singură valoare
fizică, două măsurători consistente).  Folosim **f_px_raw = 578**.

### Grid-ul 40×30 folosit de flow (step-16 decimare)

```
f_px_grid_h = 20 / tan(29°)   = 36.08 px  (axa dx, 40 grid-px pentru 58°)
f_px_grid_v = 15 / tan(22.5°) = 36.21 px  (axa dy, 30 grid-px pentru 45°)
```

Ambele ~**36 grid-pixeli** focal length.  Constante în firmware:

```c
#define FLOW_FOCAL_PX_GRID   36   // post-step-16 decimation
#define FLOW_FOCAL_PX_RAW   578   // original 640×480
#define FLOW_FOV_H_DEG       58
#define FLOW_FOV_V_DEG       45
```

## Aliniere axe în body-frame (confirmată E29)

Convenția stabilită în [flow_body_frame.md](flow_body_frame.md):
- Latura lungă a senzorului (axa dx / FOV 58°) = axa **FW-BACK** a dronei
- Latura scurtă (axa dy / FOV 45°) = axa **L-R** a dronei

Deci pe axa FW-BACK avem mai multă acoperire unghiulară, ceea ce e
util: la mișcarea înainte (viteza dominantă la un drone în flight)
flow-ul e mai sensibil.

## Formula altitudinii

Pentru o cameră perpendiculară pe sol, la altitudine `h`, dronă
translând cu viteză orizontală `v` (m/s), un punct static din scenă
se deplasează în imagine cu:

```
pixel_rate = v * f_px / h    (pixeli/secundă)
```

Rezultă:

```
h = v * f_px / pixel_rate
```

Pe axele body-frame (cam0, unde dx opusă FW):

```
h_from_fw_motion   = |v_fw|   * f_px_grid_h / |dx_per_s|
h_from_left_motion = |v_left| * f_px_grid_v / |dy_per_s|
```

Cele două estimări trebuie să concorde (dacă nu, fie viteza e greșită,
fie scena nu e la altitudine uniformă).  Media lor e un estimator mai
robust:

```
h = (h_fw + h_left) / 2    # când ambele sunt valide
```

## Surse de viteză orizontală

Pentru a calcula altitudinea avem nevoie de `v_horiz` din altă sursă
decât flow (altfel ambiguitate).  Opțiuni:

### 1. IMU accel-integrated (LIS2DU12 pe board)

- Citește accel 100-200 Hz, substractă gravitația (sau high-pass
  filter), integrează pentru viteză
- **Drift**: integrarea dublă a zgomotului → drift exponențial
- **Fix**: fereastră scurtă (0.5-2 s) cu reset periodic din flow
  (complementary filter)
- **Precizie**: ±0.1-0.3 m/s pe durate sub 1 s (necalibrat)

### 2. Velocitate cunoscută în experiment (calibrare)

- Utilizatorul mișcă drona pe o distanță măsurată (regla) în timp
  măsurat (cronometru)
- `v = Δx_ruler / Δt`
- Bun doar pentru calibrare/validare

### 3. Odometrie din autopilot (PX4 / ArduPilot)

- Autopilot are deja `VFR_HUD.groundspeed` sau similar
- Feed către firmware prin UART/MAVLink
- În aplicații reale asta e sursa

## Propunere arhitectură altitudine live

```
┌──────────┐    v_horiz  ┌────────────────┐
│   IMU    │───────────▶│ altitude_fuse  │───▶ h_estim
│ (LIS2DU) │   (m/s)     │  (M4 sau M7)   │    (m AGL)
└──────────┘             └───────▲────────┘
                                 │  dx,dy,dt
                                 │
                         ┌──────────────┐
                         │ sentai.flow  │
                         │    (M4)      │
                         └──────────────┘
```

Nivel 1 (manual / calibrare): flow + viteza furnizată din afară.
Nivel 2 (autonomy): flow + IMU + complementary filter la 20 Hz.
Nivel 3 (redundanță): adaugă VL53L0X ToF I2C pentru groundtruth AGL.

## Precizie așteptată

Cu `dx_grid = 1` pixel pe frame la 25 fps, `dt = 40 ms`:

| `|v_horiz|` | `pixel_rate` | `h` estimat | Eroare ±1px |
|---|---|---|---|
| 0.1 m/s | 0.36 px/s | 10 m | ±50% (coarse) |
| 0.5 m/s | 4.5 px/s  | 4 m | ±22% |
| 1.0 m/s | 18 px/s   | 2 m | ±6% |
| 2.0 m/s | 36 px/s   | 1 m | ±3% |

Cu cât drona merge mai repede, cu atât e mai precisă estimarea — dar
la viteze mari riscă să depășească fereastra SAD (±6 px = ±96 raw).
**Optim pentru estimare altitudine**: dronă la 0.5-2 m/s, care dă
flow de 3-30 px/s cu eroare 5-20%.
