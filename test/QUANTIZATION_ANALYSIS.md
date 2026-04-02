# Analiza cuantizare EdgeTPU — yolo26n.edgetpu.tflite

**Data:** 2 Aprilie 2026  
**Modele analizate:** yolo26n.tflite, yolo26n_v2.tflite, yolo26n.pb, yolo26n.edgetpu.tflite  
**Imagine test:** input.jpeg (320×320, fork + knife + cup pe masă)  
**Hardware:** Coral USB Accelerator (ID 18d1:9302), libedgetpu1-std 16.0

---

## 1. Comparație modele

| Model | Input | Output | NMS | Precizie |
|---|---|---|---|---|
| yolo26n.tflite | float32 [1,320,320,3] | [1,300,6] post-NMS | Built-in | float32 |
| yolo26n_v2.tflite | float32 [1,320,320,3] | [1,300,6] post-NMS | Built-in | float32 |
| yolo26n.pb | float32 [1,320,320,3] | [1,300,6] post-NMS | Built-in | float32 |
| yolo26n.edgetpu.tflite | int8 [1,320,320,3] | [1,84,2100] raw YOLO | Manual (CPU) | int8 |

**yolo26n.tflite și yolo26n_v2.tflite sunt identice funcțional** — aceeași arhitectură (700 tensori),
aceleași detecții, diferă doar numerotarea concat-urilor și 416 bytes.

## 2. Rezultate detecție pe input.jpeg

### TFLite (float32) — 10 detecții corecte
```
[53,94,169,246]   21.3% fork       ✓
[0,134,29,223]    11.4% toaster
[167,25,242,139]  10.7% cup        ✓
[-8,2,319,322]    10.6% laptop
[167,25,242,139]   9.3% bottle     ✓
[94,62,203,238]    8.5% knife      ✓
[53,94,169,246]    8.5% carrot
[272,0,320,92]     7.0% cup        ✓
[94,62,203,238]    6.7% carrot
[0,0,71,62]        5.0% remote
```

### EdgeTPU (int8) — 6 detecții GREȘITE (toate "tie")
```
[0,139,28,222]    64.3% tie        ✗
[276,1,320,88]    49.9% tie        ✗
[0,0,73,59]       42.7% tie        ✗
[5,89,211,245]    23.3% tie        ✗
[1,0,169,100]     13.9% tie        ✗
[167,26,240,137]  11.1% tie        ✗
```

## 3. Pipeline-ul intern al modelului EdgeTPU

```
tensor[0]  input          [1,320,320,3]  int8   sc=0.019  zp=-14
           ↓
    [edgetpu-custom-op]   ← backbone complet pe EdgeTPU chip
           ↓
tensor[3]  conv output    [1,40,40,80]   int8   sc=0.299  zp=119
           ↓ Transpose, Reshape, Concat (CPU)
tensor[8]  class logits   [1,80,2100]    int8   sc=0.299  zp=119   ← PROBLEMA
           ↓ Sigmoid (CPU)
tensor[9]  sigmoid out    [1,80,2100]    int8   sc=0.004  zp=-128
           ↓ Quantize (CPU, re-quantizare)
tensor[10] cls requant    [1,80,2100]    int8   sc=0.006  zp=-128  ← pierdere #2
tensor[11] bbox part1     [1,2,2100]     int8   sc=0.006  zp=-128
tensor[12] bbox part2     [1,2,2100]     int8   sc=0.006  zp=-128
           ↓ Concatenation (CPU)
tensor[13] output         [1,84,2100]    int8   sc=0.006  zp=-128
           layout: [bbox(4), cls(80)] × 2100 anchors
           bbox = cx, cy, w, h (normalizate 0-1)
           cls = post-sigmoid (probabilități)
```

## 4. Cauza principală: backbone-ul distruge clasificarea

### Logit-uri pre-sigmoid (tensor[8])
- **Range posibil:** [-73.9, +2.4] (dat de sc=0.299, zp=119)
- **Range actual:**  [-73.9, **-0.9**] — **TOATE logit-urile sunt negative**
- Maximul -0.9 → sigmoid(-0.9) = 0.29 — asta e maximul pe care modelul îl poate produce
- **99.8%** din valorile int8 ale claselor sunt **-128** (= zero după dequantizare)
- Doar **254 din 2100** anchors au măcar o clasă nenulă

### Clase specifice — logit int8 direct din backbone

| Clasă | Logit int8 | Sigmoid | Prezentă în imagine? |
|---|---|---|---|
| fork | -128 (SATURAT) | 0.000 | DA ✓ |
| cup | -128 (SATURAT) | 0.000 | DA ✓ |
| knife | -127 | 0.000 | DA ✓ |
| bottle | -128 (SATURAT) | 0.000 | DA ✓ |
| tie | -128 (SATURAT) | 0.000 | NU |

**Concluzie:** Chiar dacă citim logit-urile int8 direct (fără sigmoid, fără re-quantizare),
fork/cup/knife/bottle sunt la minimum absolut (-128). Problema este **în backbone**
(convoluțiile quantizate din edgetpu-custom-op), nu în post-processing.

### Ce clase supraviețuiesc quantizării?

Top clase din logit-uri (tensor[8]) cu sigmoid aplicat în float32:
```
motorcycle  logit_int8=116  sigmoid=0.289
bicycle     logit_int8=65   sigmoid=0.000
car         logit_int8=61   sigmoid=0.000
```

Doar obiectele **foarte mari** (vehicule) au logit-uri non-minime. Clasele de obiecte
mici (bucătărie: fork, cup, knife) sunt complet pierdute.

## 5. Problema secundară: re-quantizarea la output

```
tensor[9]  sigmoid:  scale=0.003906  → 256 niveluri [0..1]  ✓ OK
tensor[10] requant:  scale=0.005538  → 180 niveluri [0..1]  ~ acceptabil
tensor[13] output:   scale=0.005538  → SHARED cu bbox
```

- Bbox-urile au range [0..1.35], forțând scale=0.005538 pe tot output-ul
- Clasele (care sunt [0..1]) pierd ~30% din rezoluție din cauza shared scale
- Un scor de 0.05 = doar **9 nivele int8** deasupra lui zero
- Asta nu ar fi fatală singură, dar amplifica degradarea din backbone

## 6. Bug găsit: EdgeTPU suprascrie input buffer

**Simptom:** Rezultate diferite între invoke-uri consecutive (alternare par/impar).

**Cauza:** EdgeTPU-ul modifică buffer-ul de input la `invoke()`. Dacă nu re-setezi
`set_tensor()` înainte de fiecare invoke, al doilea invoke primește date corupte.

**Fix aplicat în test_edgetpu.py:**
```python
# Warmup (EdgeTPU may overwrite the input buffer, so re-set before real run)
interp.set_tensor(inp['index'], input_data.reshape(inp['shape']))
interp.invoke()

interp.set_tensor(inp['index'], input_data.reshape(inp['shape']))  # ← NECESAR
interp.invoke()  # real run
```

## 7. Soluții propuse

### 7.1 Re-export cu calibrare mai bună (cel mai simplu)

```python
def representative_dataset():
    # Trebuie 100-500 imagini similare cu cele de producție
    # NU doar imagini random de pe internet
    for img_path in calibration_images:
        img = preprocess(img_path)  # 320x320, float32, /255
        yield [img]

converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_dir)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8
tflite_quant = converter.convert()
```

**Important:** Dataset-ul de calibrare trebuie să conțină imagini cu obiectele
pe care le va vedea modelul (bucătărie, unelte). Dacă calibrarea s-a făcut cu
imagini de stradă, doar clasele de vehicule supraviețuiesc.

### 7.2 Quantization-Aware Training (QAT) — cea mai bună soluție

Necesită modelul original (Keras/PyTorch) + dataset de antrenare. Se re-antrenează
cu fake quantization nodes care simulează pierderea int8:

```python
import tensorflow_model_optimization as tfmot

# Aplică QAT pe modelul Keras
qat_model = tfmot.quantization.keras.quantize_model(float_model)
qat_model.compile(optimizer='adam', loss=...)
qat_model.fit(train_dataset, epochs=5)  # fine-tune scurt

# Export quantized
converter = tf.lite.TFLiteConverter.from_keras_model(qat_model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_qat = converter.convert()
```

### 7.3 Split model (backbone EdgeTPU + heads CPU float32)

- Export doar convoluțiile grele ca int8 pentru EdgeTPU
- Detection heads (cls + bbox final conv) rămân float32 pe CPU
- EdgeTPU face ~70% din compute, CPU face restul cu precizie deplină
- Necesită export custom cu output intermediar din backbone

### 7.4 Separate outputs (fără concat cls+bbox)

- 2 output tensors: `bbox[1,4,2100]` și `cls[1,80,2100]`
- Fiecare cu scale/zp propriu, optimizat pentru range-ul său
- Nu rezolvă problema de backbone, dar elimină pierderea de 30% la requant
