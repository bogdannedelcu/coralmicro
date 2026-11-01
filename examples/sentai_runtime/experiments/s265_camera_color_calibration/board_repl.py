"""Minimal REPL driver for the SentAI board (EXP-s265).

Prompt-synchronised: every command waits for the next ``>>> `` prompt instead of a
fixed sleep.  The embed MicroPython has no ``time`` module, so waits between
camera operations are done on the host (``time.sleep``) between commands.
"""
import base64
import glob
import time

import serial


class Repl:
    def __init__(self, port=None, baud=115200, log=None):
        self.port = port or (sorted(glob.glob('/dev/ttyACM*')) or ['/dev/ttyACM0'])[0]
        self.ser = serial.Serial(self.port, baud, timeout=0.2)
        self.log = log
        self.ser.write(b'\r\x03\x03\r')
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.cmd('import sentai')

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def cmd(self, line, timeout=20.0):
        """Send one line, return everything printed before the next prompt."""
        self.ser.reset_input_buffer()
        self.ser.write((line + '\r').encode())
        buf = b''
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
                if buf.endswith(b'>>> '):
                    break
        out = buf.decode(errors='replace')
        # strip the echoed command and the trailing prompt
        if out.startswith(line):
            out = out[len(line):]
        out = out.rstrip()
        if out.endswith('>>>'):
            out = out[:-3].rstrip()
        if self.log:
            self.log.write('>>> %s\n%s\n' % (line, out))
            self.log.flush()
        return out.strip('\r\n')

    def value(self, expr, timeout=20.0):
        """Evaluate ``expr`` on the board and return the printed last line."""
        out = self.cmd('print(%s)' % expr, timeout)
        lines = [l for l in out.replace('\r', '').split('\n') if l.strip()]
        return lines[-1].strip() if lines else ''

    def download(self, path, timeout=60.0):
        """Fetch a file (<= 256 KB) through ``sentai.fs.read_base64``."""
        out = self.cmd('sentai.fs.read_base64("%s")' % path, timeout)
        b64 = ''.join(l.strip() for l in out.replace('\r', '').split('\n')
                      if l.strip() and not l.startswith("'") and ' ' not in l.strip())
        return base64.b64decode(b64)

    # --- camera helpers -------------------------------------------------
    def select(self, cam, settle_s=1.2):
        r = self.value('sentai.camera.select(%d)' % cam)
        time.sleep(settle_s)
        return r

    def reg(self, cam, addr):
        v = self.value('sentai.camera.reg_read(%d, %d)' % (cam, addr))
        return int(v)

    def regw(self, cam, addr, val):
        return int(self.value('sentai.camera.reg_write(%d, %d, %d)' % (cam, addr, val)))

    def regs(self, cam, addrs):
        v = self.value('[sentai.camera.reg_read(%d, r) for r in (%s,)]'
                       % (cam, ','.join(str(a) for a in addrs)))
        return [int(x) for x in v.strip('[]').split(',')]

    def snap(self, cam, path, quality=90, settle_s=1.2):
        """Select ``cam``, wait, save a JPEG on the board and return its bytes."""
        self.select(cam, settle_s)
        return self.snap_cur(path, quality)

    def snap_cur(self, path, quality=90):
        """Save a JPEG from the currently selected camera and return its bytes."""
        n = int(self.value('sentai.camera.save_jpeg("%s", %d)' % (path, quality), 30))
        if n <= 0:
            raise RuntimeError('save_jpeg(%s) -> %d' % (path, n))
        data = self.download(path)
        if len(data) != n:
            raise RuntimeError('download %s: %d of %d bytes' % (path, len(data), n))
        self.cmd('sentai.fs.remove("%s")' % path)
        return data
