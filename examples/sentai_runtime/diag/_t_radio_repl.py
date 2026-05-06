# _t_radio_repl.py -- on-board listener for CPX APP messages from the radio.
#
# Pattern B: firmware queues each APP packet from the host PC; this driver
# polls the queue and dispatches via process_message(). REPL one-liners
# prefixed with b'>>> ' are exec'd on the board and the result (or error)
# is sent back to the host over the same APP channel.
#
# Usage from REPL:
#   import sentai
#   sentai.crazy.init()
#   import diag._t_radio_repl as r
#   r.run()                  # 60s default
#   r.run(secs=600)          # 10 min listening
#
# Wire format (host <-> board, payload of CPX function=APP):
#   bytes starting with b'>>> '  : MicroPython source line to exec on board
#   anything else                 : free-form text logged on board console
# Response (board -> host):
#   b'OK\n'                       : exec succeeded with no value
#   b'OK ' + repr(value)          : exec succeeded with a value (eval form)
#   b'ERR ' + str(exception)      : exec raised
import sentai

PROMPT = b'>>> '


def process_message(msg):
    """Handle one APP message from the host radio. Returns reply bytes or None."""
    # MicroPython doesn't expose `bytearray` as a builtin name in this
    # firmware build, so check `bytes` only — that's what the queue
    # always returns anyway.
    if not isinstance(msg, bytes):
        return None
    if msg.startswith(PROMPT):
        src = bytes(msg[len(PROMPT):]).decode('utf-8', 'replace').strip()
        if not src:
            return b'ERR empty'
        # Try eval-form first (so '1+1' returns '2'); fall back to exec.
        try:
            try:
                code = compile(src, '<radio>', 'eval')
                val = eval(code, globals(), globals())
                return b'OK ' + repr(val).encode('utf-8', 'replace')[:80]
            except SyntaxError:
                code = compile(src, '<radio>', 'exec')
                exec(code, globals(), globals())
                return b'OK\n'
        except Exception as e:
            return (b'ERR ' + (type(e).__name__ + ': ' + str(e))
                    .encode('utf-8', 'replace'))[:90]
    # Plain text: log it locally, no reply.
    try:
        print('[radio] msg:', bytes(msg).decode('utf-8', 'replace'))
    except Exception:
        print('[radio] bin:', bytes(msg))
    return None


def run(secs=60, poll_ms=200):
    """Drain the CPX APP queue for `secs` seconds, dispatching each message."""
    print('[radio] listening %d s ...' % secs)
    n_in = 0
    n_out = 0
    t_end = sentai.rtos.ticks_ms() + secs * 1000
    while sentai.rtos.ticks_ms() < t_end:
        msg = sentai.crazy.poll_event(poll_ms)
        if msg is None:
            continue
        n_in += 1
        reply = process_message(msg)
        if reply is not None:
            try:
                sentai.crazy.send_app(reply)
                n_out += 1
            except Exception as e:
                print('[radio] send_app EXC:', e)
    print('[radio] done  in=%d out=%d' % (n_in, n_out))
    return n_in
