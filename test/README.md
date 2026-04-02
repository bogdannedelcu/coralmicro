# Test YOLO26n — PC-side inference

Testare pe PC a celor 3 formate ale modelului **yolo26n** (COCO 80 clase, input 320×320).

## Modele

| Fișier | Format | Input | Output | Rulează pe |
|--------|--------|-------|--------|------------|
| `yolo26n.tflite` | TFLite float32 | `[1,320,320,3]` float32, **0-1** | `[1,300,6]` float32 — post-NMS, coords normalizate 0-1 | CPU |
| `yolo26n.pb` | TF Frozen GraphDef | `[1,320,320,3]` float32, **0-1** | `[1,300,6]` float32 — post-NMS, coords în **pixeli** | CPU |
| `yolo26n.edgetpu.tflite` | TFLite int8 + EdgeTPU | `[1,320,320,3]` int8, `pixel + zp` | `[1,84,2100]` int8 — raw YOLO, **fără NMS** | Coral USB/PCIe |

### Diferențe importante între modele

- **tflite** și **pb**: au NMS built-in (output = 300 detecții sortate).
  Coordonatele sunt normalizate (tflite) sau în pixeli (pb).
- **edgetpu**: output **raw** `[C, N]` unde C=84 (4 bbox + 80 clase), N=2100 anchors.
  NMS trebuie făcut manual (portat din `sentai_runtime.cc` → `sentai_tpu_detect()`).

---

## Medii virtuale (venv)

### `venv/` — pentru TFLite CPU + PB (Python 3.12)

```bash
python3.12 -m venv venv
source venv/bin/activate
pip install tensorflow numpy Pillow opencv-python-headless
```

| Pachet | Versiune testată |
|--------|-----------------|
| Python | 3.12.3 |
| tensorflow | 2.21.0 |
| numpy | 2.4.4 |
| Pillow | 12.2.0 |
| opencv-python-headless | 4.13.0.92 |

> **Nu funcționează** cu Coral USB Accelerator!
> `tf.lite` din TF 2.21 are incompatibilitate cu `libedgetpu 16.0`.

### `venv_coral/` — pentru EdgeTPU pe Coral USB (Python 3.9)

```bash
# Necesită Python 3.9 (pycoral/tflite-runtime nu suportă 3.12)
sudo apt install python3.9 python3.9-venv
python3.9 -m venv venv_coral
source venv_coral/bin/activate
pip install numpy==1.26.4 Pillow
pip install tflite-runtime==2.5.0.post1 \
    --extra-index-url https://google-coral.github.io/py-repo/
pip install pycoral==2.0.0 \
    --extra-index-url https://google-coral.github.io/py-repo/
```

| Pachet | Versiune testată |
|--------|-----------------|
| Python | 3.9.25 |
| tflite-runtime | 2.5.0.post1 |
| pycoral | 2.0.0 |
| numpy | 1.26.4 (≤1.x obligatoriu, numpy 2.x incompatibil) |
| Pillow | 11.3.0 |

### Dependență sistem: `libedgetpu`

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
    | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
wget -qO- https://packages.cloud.google.com/apt/doc/apt-key.gpg \
    | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/coral-edgetpu.gpg
sudo apt update
sudo apt install libedgetpu1-std    # standard speed
# sau: sudo apt install libedgetpu1-max   # max speed (mai cald)
```

După instalare, reload udev + replug Coral USB:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Scripturi de test

| Script | Model | venv | Comandă |
|--------|-------|------|---------|
| `test_tflite.py` | `yolo26n.tflite` | `venv/` | `source ../venv/bin/activate && python test_tflite.py [conf] [image]` |
| `test_pb.py` | `yolo26n.pb` | `venv/` | `source ../venv/bin/activate && python test_pb.py [conf] [image]` |
| `test_edgetpu.py` | `yolo26n.edgetpu.tflite` | `venv_coral/` | `source ../venv_coral/bin/activate && python test_edgetpu.py [conf] [image]` |

### Argumente (comune)

```
python test_*.py [confidence_threshold] [image_path]
```

- `confidence_threshold` — default `0.05` (5%). Pe sentai se folosește valoare mică.
- `image_path` — default `input.jpeg`

### Output

Fiecare script salvează o imagine annotată:
- `det_tflite.jpg`, `det_pb.jpg`, `det_edgetpu.jpg`

Stilul de desenare este identic cu `sentai_tpu_draw()`:
- Bounding box 2px, 10 culori ciclice (`kBoxColors`)
- Label cu font mic, background colorat
- Clase COCO (80)

---

## Algoritm NMS (doar EdgeTPU)

Modelul EdgeTPU produce output raw `[1, 84, 2100]` — fără NMS.
Scriptul `test_edgetpu.py` implementează NMS identic cu `sentai_tpu_detect()`:

1. **Dequantizare**: `float_val = scale × (int8_val − zero_point)`
2. **Extragere candidați**: pentru fiecare din 2100 anchors, găsește clasa cu scor maxim
3. **Filtrare confidență**: păstrează doar candidații cu `score > conf_threshold`
4. **Auto-detect coordonate**: dacă `max_coord < 2.0` → normalizate, altfel pixeli
5. **Conversie bbox**: `cx,cy,w,h` → `x1,y1,x2,y2`
6. **Sortare** descrescător după scor
7. **Greedy NMS class-aware**: suppress overlapping same-class boxes cu `IoU > iou_threshold`

---

## Observații

- Modelul `yolo26n` e mic — confidențele sunt în general **mici** (5-25%).
  Nu te aștepta la 90%+ ca la YOLO standard.
- Modelul EdgeTPU int8 quantized produce rezultate **ușor diferite** de float32:
  clasele pot diferi (ex: "tie" în loc de "fork") din cauza pierderii de precizie la quantizare.
- Input-ul este **0-1 normalizat** (float) sau **pixel + zero_point** (int8), **NU** 0-255 brut.
