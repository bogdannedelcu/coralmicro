# SentAI Optical-Flow → Drone Body-Frame Convention

**Last verified**: 2026-04-21 via experiment E29 (sesiunile
`s064_e29_orientation`, `s065_e29_orientation_readable`) cu o foaie
A4 desenată manual cu săgeți FW / BACK / L / R, ținută în fața dronei,
capturată succesiv de cam0 și cam1.

## Convenție body-frame

Drona folosește convenția standard **body-frame North-East-Down
(inverted)** a autopilot-urilor de tip PX4 / ArduPilot:

| Axă drona | Descriere | Direcție pozitivă |
|-----------|-----------|-------------------|
| `X = FW`  | roll / forward | înainte (nasul dronei) |
| `Y = L`   | pitch / lateral | stânga |
| `Z = UP`  | yaw / vertical | sus |

Toate ieșirile `sentai.flow` feedate la autopilot trebuie să
respecte semnele acestor axe.

## Montaj fizic senzori pe Coral Dev Board Micro

Board-ul are două module OV5640 conectate prin MIPI CSI-2 printr-un
MUX analog:

- **cam0** (front): FFC pe o față a board-ului
- **cam1** (back): FFC pe fața opusă

Observațiile cheie de hardware (E29):

1. **Fiecare senzor individual e montat rotit 90°** relativ la
   orientarea "sus" a board-ului.  Nu se poate corecta prin
   registrii OV5640 (pot doar H-mirror, V-flip sau combinația
   180°); pentru "up natural" trebuie rotație PXP hardware.
2. **Cele două camere sunt rotite 180° una față de alta** —
   efect cumulat al montajului pe fețe opuse + orientarea ținerii
   senzorilor în package.
3. NXP OV5640 init setează `0x3821 = 0x07` (H-mirror sensor + ISP)
   — e păstrat ca default, altfel textul apare oglinda speculară.

## Mapare direcție drona → pixel în imagine

Ținând o foaie perpendicular pe fiecare cameră (E29), cu săgețile
orientate FW up / BACK down / L left / R right în sistemul de
coordonate al dronei:

### Cam0 (front camera)

```
          IMG-TOP = L (drona)
          ↑
IMG-LEFT  ╳  IMG-RIGHT
BACK    cam0     FW
          ↓
          IMG-BOT = R (drona)
```

| Zonă imagine | Direcție drona |
|---|---|
| RIGHT | FW |
| LEFT | BACK |
| TOP | L |
| BOTTOM | R |

### Cam1 (back camera, 180° față de cam0)

```
          IMG-TOP = R (drona)
          ↑
IMG-LEFT  ╳  IMG-RIGHT
FW     cam1    BACK
          ↓
          IMG-BOT = L (drona)
```

| Zonă imagine | Direcție drona |
|---|---|
| LEFT | FW |
| RIGHT | BACK |
| TOP | R |
| BOTTOM | L |

## Optical-flow sign convention

`sentai.flow.m4_read()` întoarce `(dx, dy)` în **grid 40×30** (după
decimarea step-16 din raw 640×480), unde:

- `dx > 0`: scena s-a deplasat spre **dreapta** imaginii
- `dy > 0`: scena s-a deplasat în **jos** pe imagine

Când drona se mișcă într-o direcție, scena se deplasează în
**sensul opus** pe senzor.  Combinând cu maparea de mai sus:

### Cam0

```python
body_fw   = -dx    # FW e în dreapta → scena merge stânga → dx negativ
body_left = +dy    # L e sus → scena merge jos → dy pozitiv
```

### Cam1 (180° rotit)

```python
body_fw   = +dx    # FW e în stânga → scena merge dreapta → dx pozitiv
body_left = -dy    # L e jos → scena merge sus → dy negativ
```

## Unități

Grid-ul 40×30 din raw 640×480 → **fiecare pixel grid = 16 px raw**
pe ambele axe.  Pentru a converti în **velocitate body-frame**
integrată peste un frame (ex. 40 ms la 25 fps):

```
body_fw_px_per_frame = body_fw_grid × 16
body_fw_m_per_frame  = body_fw_px_per_frame × scene_m_per_px
```

`scene_m_per_px` depinde de distanța scenei și de FOV-ul camerei
(OV5640 VGA are FOV diagonal ~65°, orizontal ~55°).  La 1 m de la
subiect, 1 px ≈ 1.6 mm pe axa orizontală.

## Recomandare pentru integrarea cu autopilot

**Folosește cam0 constant** (`sentai.camera.select(0)` + `ratio(0,0)`).
Switching între cam0 și cam1 implică flip de semne la `body_fw` și
`body_left` — dacă autopilot-ul nu e informat de switch, integrează
ca o săritură bruscă în poziție, care se traduce în perturbare
agresivă de control.

Dacă e nevoie de redundanță (failover la cam1), autopilot-ul
trebuie să:
1. Primească un flag `cam_id` odată cu fiecare măsurătoare
2. Aplice transformarea corectă pe baza `cam_id`
3. La switch, re-inițializeze integratorul de poziție pentru a
   evita săriturile

Alternativ: aplică conversia body-frame pe firmware (în
`modsentai_flow.c` — proposal: `sentai.flow.m4_body_read()`),
autopilot-ul primește întotdeauna `(body_fw, body_left)`
indiferent de camera activă.
