# linkproxy (Python)

Python app that connects to a MAVLink router over TCP, reads MAVLink messages, reconstructs chunked `STATUSTEXT` payloads, decodes `VisionMessage` packets sent by `sentai.link.send_detection()` / `send_update()`, and forwards them to MQTT.

## What it reads

From the project code:

- `sentai.link.send_detection(...)`
- `sentai.link.send_update(...)`

These build a `visionmesh.VisionMessage`, nanopb-encode it, base64-encode it, then split it into MAVLink `STATUSTEXT` chunks with:

- `id = msg_id`
- `chunk_seq = 0..N`
- `text[50] = base64 chunk`

See:
- `examples/sentai_runtime/sentai_link.cc`

## MQTT topics

By default, it publishes:

- `sentai/vision` for decoded VisionMessage payloads
- `sentai/text` for plain non-chunked STATUSTEXT messages
- `sentai/raw` for raw/debug payloads
- `sentai/status` for short service state messages
- `sentai/availability` for health / monitoring telemetry

## Buffering

Like `meshproxy`, it keeps a disk spool:

- `spool/pending/`
- `spool/Sent/`

Pending messages are retried when MQTT reconnects. Sent files are kept for 7 days.

## Setup

```bash
cd backend/linkproxy
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
python main.py
```
