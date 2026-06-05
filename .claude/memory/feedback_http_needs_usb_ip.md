---
name: HTTP requires explicit usb.ip(1)
description: HTTP/CDC-NCM does NOT start by itself at boot — caller must invoke sentai.usb.ip(1) over REPL before curl works.
type: feedback
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
HTTP / CDC-NCM is OFF by default at boot.  `curl http://10.0.0.1/...`
will not respond until `sentai.usb.ip(1)` is sent over the REPL.

**Why:** user clarified 2026-04-25 — the default boot brings up REPL
(CDC-ACM) only.  Network-Control-Model (CDC-NCM) is opt-in to keep
the host-visible USB topology minimal until needed.  Stale agent.md
text suggesting HTTP is up at boot is wrong.

**How to apply:** in any host-side workflow that uses HTTP (file
download, /api/ls, /api/raw), the first REPL command sent after boot
must be `sentai.usb.ip(1)` (returns 0 on success).  Then wait ~1 s for
NCM enumeration before issuing the first `curl`.  Skipping this leads
to "Connection refused" / hung curls that look like firmware bugs.
