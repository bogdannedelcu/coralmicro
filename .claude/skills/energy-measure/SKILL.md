---
name: energy-measure
description: Measure EdgeTPU inference energy on the SentAI board with an FNIRSI FNB48P USB power meter — get a model onto the board, then run a paper-grade N>=5 auto-window capture producing E_frame and energy-per-pixel with uncertainties. Use for the applsci energy paper (Energy-per-Pixel) measurements, or any on-board power/energy characterization.
---

# /energy-measure

End-to-end procedure to measure per-inference energy of an EdgeTPU model on the SentAI
CoralMicro board. Board is powered **through** the FNB48P meter; the meter is read over USB-HID
on the host. Validated 2026-07-14 (s-512: E_frame = 20.54 ± 0.24 mJ, ε_pixel = 78.37 nJ/pixel).

Scripts live in `/home/bogdan/work/appliedsci/`: `energy_capture.py` (full N-run capture),
`fnb_log.py` (raw continuous logger). They need python `hidapi` + `pyserial` (in the session
scratchpad venv, or `pip install hidapi pyserial`). FNB48P access needs **root** (hidraw).

## 0. Hardware check

- Meter: `lsusb` shows `2e3c:0049 FNIRSI USB Tester` → node `/dev/hidraw5` (map via
  `/sys/class/hidraw/*/device/uevent`, HID_ID `...2E3C:00000049`). Root-only.
- Board: `1fc9:c0a1 NXP ... SentAI` → REPL on `/dev/ttyACM0` (115200). `sentai.version()` works.
- **Only the board must be powered through the meter** (Coral USB Accelerator etc. skew the idle
  baseline). Board idle ≈ 0.77–0.81 W (5.1 V × ~0.16 A).

## 1. FNB48P protocol (baryluk/fnirsi-usb-power-data-logger)

hidapi, prepend report-id `0x00` on every write. Init: `\xaa\x81`+61×`\x00`+`\x8e`, then
`\xaa\x82`+61×0+`\x96` twice. Keepalive `\xaa\x83`+61×0+`\x9e` every ~1 s (must keep sending or
the stream stops). Data packet when `read()[1]==0x04`: 4 samples × 15 B from offset 2; per sample
V = LE-uint32[off..off+3]/1e5 (V), I = LE-uint32[off+4..off+7]/1e5 (A), T = (b[off+13]+b[off+14]*256)/10.
Native 100 Hz (4 samples/packet).

## 2. Get a model onto the board

Build 1590's FileX user partition is often in **SAFE_MODE** (`sentai.fs.exists("/")` → False). If so:

```python
sentai.fs.format()          # NO arg in build 1590 (NOT sentai.diag.fx_format(0xDEADBEEF) — that is a
                            # different build; sentai.diag does not exist here). DESTRUCTIVE to the
                            # user partition only — needs explicit operator OK. Prints lx_format ok /
                            # fx_media_format ok / mounted; then exists("/") → True.
```

Then push via USB-MSC (models are ~5–9 MB, too big for base64 REPL):

```python
sentai.usb.drive(1)         # -> USB re-enumerates, serial DROPS (expected)
```
```bash
# board appears as /dev/sdc (55 MB) — NOT /dev/sda (host system disk!). After format it is
# vfat LABEL SENTAI_USER UUID 0000-0001.
sudo mount -t vfat /dev/sdc /mnt/sentai_msc
sudo cp model_edgetpu.tflite /mnt/sentai_msc/model512.tflite
sudo sync && sudo umount /mnt/sentai_msc
```
```python
sentai.usb.drive(0)         # exit MSC (serial drops again; reconnect after ~8 s).
                            # 'q' on ttyACM0 alone leaves a half-state ("flash busy: call drive(0)").
sentai.fs.ls("/")           # verify model present; sentai.fs.exists("/model512.tflite") → True
```
MSC and FileX are mutually exclusive (see `[[fx-reformat-dev]]`, `[[fx-sync-protocol]]`).

## 3. Run inference from the REPL

```python
sentai.tpu.load("/model512.tflite")
sentai.tpu.invoke()                       # returns latency ms (e.g. 41 for s-512)
for i in range(100): sentai.tpu.invoke()  # single-line burst -> sustained power plateau
```

## 4. Capture energy (paper-grade)

```bash
sudo /path/to/venv/bin/python /home/bogdan/work/appliedsci/energy_capture.py out.csv
```
It logs 100 Hz in a background thread, runs **N=5** bursts of **K=100** invokes, auto-detects each
integration window by a **current threshold** (I0 + max(0.02, 5σ) — answers R3's manual-window bias),
trapezoidally integrates net power (over idle P0) → **E_frame = E_net/K**, **ε_pixel = E_frame/(W·H)**,
with 95% CI (statistical) ⊕ 0.5‰ instrument term. Edit `MODEL`, `W`, `H`, `N_RUNS`, `K` at the top per
model. Writes `out.csv` + `out.csv.runs` (window bounds + P0/E_frame/eps metadata) for plotting.

Plot: overlay the power trace, shade the N run windows, annotate E_frame ± CI and ε_pixel.

## Gotchas

- Keepalive is mandatory — without the periodic `\xaa\x83` write the meter stops streaming.
- `sentai.usb.drive(1/0)` each drop the serial; reconnect with a retry loop (~8 s).
- Format the FileX volume BEFORE switching to MSC, else the MSC volume is raw `0xFF` (unmountable).
- Multiple stray board activities skew idle; keep the board otherwise quiet during a capture.
- EdgeTPU latency/energy are weight-independent (fixed dataflow) — an untrained-but-compiled model
  gives the same energy as the trained one; verify the compiled graph with `inspect_edgetpu_model.py`.

## TPU clock sweep (WS0 transfer-bound test) — build 1593+

Firmware reads a one-char `/tpu_mode` file in `Main()` and opens the Edge TPU ONCE
at that clock per boot: `'0'`=kLow(63) `'1'`=kMedium(125) `'2'`=kHigh(250) `'3'`=kMax(500).
Clean per-mode measurement (NO runtime set_mode, which can wedge the USB/TPU):
```python
sentai.fs.write("/tpu_mode","0"); sentai.fs.sync(); sentai.sys.reset()   # boots at kLow
# after ~13 s reconnect, sentai.tpu.load(model), then measure invoke()/energy
```
`sentai.tpu.set_mode(n)` also exists (runtime reopen) but is fragile — prefer file+reboot.
Measured s-512 latency vs clock: kMax 41 / kHigh 49 / kMedium 66 / kLow 100 ms →
fit τ = 32.4 (transfer, fixed) + 4251/f_MHz (compute); 79% of the kMax invoke is USB
transfer. Clean-boot values match set_mode → the clock genuinely changes.
