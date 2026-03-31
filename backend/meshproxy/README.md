# meshproxy (Python)

Python app that connects over USB serial to a WishMesh/Meshtastic-compatible device and forwards received messages to MQTT.

## What it does

- opens the USB serial port
- parses Meshtastic-style serial frames
- decodes VisionMessage payloads sent by `sentai.mesh.send_detection()` / `send_update()`
- forwards text and vision messages to MQTT
- if MQTT is down, stores outgoing messages on disk in a pending buffer
- when MQTT comes back, flushes pending messages in order
- after successful publish, moves each buffered file into `Sent/`
- automatically deletes files from `Sent/` older than 7 days
- publishes periodic availability telemetry to MQTT

## Availability telemetry

The proxy publishes periodic telemetry on `sentai/availability` that includes:

- configured serial port
- serial baud
- currently detected serial ports on the machine
- whether any mesh message was received over USB in the last 10 minutes
- system indicators:
  - hostname
  - IP addresses
  - OS/platform
  - Python version
  - CPU core count
  - load average (1m/5m/15m)
  - memory total/available/used
  - disk usage for `/`
  - uptime

Short lifecycle state messages still go to `sentai/status`.

## Disk buffer behavior

Spool structure:

- `spool/pending/` - messages waiting to be published to MQTT
- `spool/Sent/` - messages already published successfully

Rules:

- if MQTT is unavailable, messages are written to `pending/`
- when MQTT reconnects, pending messages are retried in order
- only after a successful publish does a file move from `pending/` to `Sent/`
- files in `Sent/` are kept for 7 days, then removed automatically

## Why this matches the CoralMicro side

In this repo, CoralMicro sends text over mesh via:

- `sentai.mesh.send(text, dest=0xFFFFFFFF, channel=0, ack=1)`

And sends vision detections by encoding `visionmesh.VisionMessage` into `meshtastic.Data.payload` with:

- `data.portnum = meshtastic_PortNum_PRIVATE_APP`

See:
- `examples/sentai_runtime/modules/sentai/modsentai.c`
- `examples/sentai_runtime/sentai_mesh.cc`
- `third_party/meshtastic/protobufs/visionmesh.proto`

## Files

- `main.py` - main app
- `requirements.txt` - Python deps
- `.env.example` - config example

Main useful config knobs:
- `MQTT_VISION_TOPIC=sentai/vision`
- `MQTT_TEXT_TOPIC=sentai/text`
- `MQTT_RAW_TOPIC=sentai/raw`
- `MQTT_STATUS_TOPIC=sentai/status`
- `MQTT_AVAILABILITY_TOPIC=sentai/availability`
- `SPOOL_ROOT=...` to relocate on-disk buffer

## Setup

```bash
cd backend/meshproxy
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
python main.py
```

## MQTT topics

By default, it publishes:

- `sentai/vision` for decoded VisionMessage payloads
- `sentai/text` for decoded text payloads
- `sentai/raw` for raw/debug payloads
- `sentai/status` for short service state messages
- `sentai/availability` for health / monitoring telemetry
