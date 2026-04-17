# diag/_session.py — session manager
# Provides: Session, begin, end, status, _save_path, _record

import sentai
from diag._util import ensure_dir, snapshot_heap


_session = None  # active session state


class Session:
    def __init__(self, name, sid):
        self.name = name
        self.sid = sid
        self.tag = "s%03d_%s" % (sid, name)
        self.dir = "/diags/%s" % self.tag
        self.seq = 0
        self.log = []  # list of (seq, experiment, csv_path, summary_line)
        self.t_start = sentai.rtos.uptime()
        self.heap_start = sentai.rtos.heap_info()

    def next_path(self, experiment):
        """Return next CSV path inside session dir."""
        self.seq += 1
        fname = "%03d_%s.csv" % (self.seq, experiment)
        return "%s/%s" % (self.dir, fname)

    def record(self, experiment, csv_path, summary_line=""):
        """Log an experiment completion."""
        self.log.append((self.seq, experiment, csv_path, summary_line))


def _next_session_id():
    """Read and increment persistent counter in /diags/.counter."""
    ensure_dir("/diags")
    counter_path = "/diags/.counter"
    sid = 1
    if sentai.fs.exists(counter_path):
        try:
            raw = sentai.fs.read_str(counter_path)
            sid = int(raw.strip()) + 1
        except:
            sid = 1
    sentai.fs.write(counter_path, str(sid))
    return sid


def begin(name="experiment"):
    """Start a new diagnostics session.
    Creates /diags/sNNN_name/ directory.
    All subsequent experiments save into this directory.
    """
    global _session
    if _session is not None:
        print("WARNING: session '%s' still open — closing it first." % _session.tag)
        end()

    sid = _next_session_id()
    _session = Session(name, sid)
    sentai.fs.mkdir(_session.dir)

    sentai.fs.write("%s/manifest.csv" % _session.dir,
                    "seq,experiment,file,summary\n")

    print("=" * 50)
    print("SESSION %s started" % _session.tag)
    print("  dir: %s" % _session.dir)
    print("=" * 50)
    return _session.tag


def end():
    """Close the active session. Write final manifest and summary."""
    global _session
    if _session is None:
        print("No active session.")
        return

    elapsed = sentai.rtos.uptime() - _session.t_start
    heap_end = snapshot_heap()

    lines = ["seq,experiment,file,summary"]
    for seq, exp, path, summ in _session.log:
        lines.append("%d,%s,%s,%s" % (seq, exp, path, summ.replace(",", ";")))
    sentai.fs.write("%s/manifest.csv" % _session.dir, "\n".join(lines) + "\n")

    summary_lines = [
        "session: %s" % _session.tag,
        "experiments: %d" % len(_session.log),
        "duration_ms: %d" % elapsed,
        "heap_start_rtos_free: %d" % _session.heap_start["rtos_free"],
        "heap_end_rtos_free: %d" % heap_end["rtos_free"],
        "heap_start_gc_used: %d" % _session.heap_start["gc_used"],
        "heap_end_gc_used: %d" % heap_end["gc_used"],
    ]
    sentai.fs.write("%s/summary.txt" % _session.dir,
                    "\n".join(summary_lines) + "\n")

    print("=" * 50)
    print("SESSION %s closed" % _session.tag)
    print("  experiments: %d" % len(_session.log))
    print("  duration: %d ms" % elapsed)
    print("  dir: %s" % _session.dir)
    print("=" * 50)

    tag = _session.tag
    _session = None
    return tag


def status():
    """Show active session status."""
    if _session is None:
        print("No active session. Call diag.begin('name') to start.")
        return
    elapsed = sentai.rtos.uptime() - _session.t_start
    print("Session: %s  |  experiments: %d  |  elapsed: %d ms" % (
        _session.tag, len(_session.log), elapsed))
    for seq, exp, path, _ in _session.log:
        print("  %03d  %s" % (seq, exp))


def _save_path(experiment):
    """Get save path: use session dir if active, else /diags/ flat."""
    if _session:
        return _session.next_path(experiment)
    ensure_dir("/diags")
    return "/diags/%s.csv" % experiment


def _record(experiment, csv_path, summary_line=""):
    """Record experiment in session log if active."""
    if _session:
        _session.record(experiment, csv_path, summary_line)
