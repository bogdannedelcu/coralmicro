import System
from System import Array, Byte, UInt64
from System.Diagnostics import Stopwatch
from System.IO import File

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 32
    sw = Stopwatch.StartNew()
    log_path = "/tmp/sentai_emu_pxp_accel.log"

    try:
        if File.Exists(log_path):
            File.Delete(log_path)
    except Exception:
        pass

    def now_ms():
        try:
            return int(sw.ElapsedMilliseconds)
        except Exception:
            return 0

    def log_line(text):
        try:
            File.AppendAllText(log_path, "t=" + str(now_ms()) + "ms " +
                               text + "\n")
        except Exception:
            pass

    def read_guest(ptr, length):
        return sysbus.ReadBytes(UInt64.Parse(str(ptr)), int(length))

    def write_guest(ptr, data):
        sysbus.WriteBytes(data, UInt64.Parse(str(ptr)))

    def get_byte(arr, idx):
        return int(arr[idx]) & 0xFF

    def area_scale_rgb(src, src_w, src_h, dst_w, dst_h, src_fmt, dst_fmt):
        out_bpp = 1 if dst_fmt == 2 else 3
        out = Array.CreateInstance(Byte, dst_w * dst_h * out_bpp)
        src_bpp = 3 if src_fmt == 1 else 4
        for oy in range(dst_h):
            y0 = (oy * src_h) // dst_h
            y1 = ((oy + 1) * src_h) // dst_h
            yh = y1 - y0 if y1 > y0 else 1
            for ox in range(dst_w):
                x0 = (ox * src_w) // dst_w
                x1 = ((ox + 1) * src_w) // dst_w
                xw = x1 - x0 if x1 > x0 else 1
                n = xw * yh
                sr = 0
                sg = 0
                sb = 0
                for sy in range(y0, y1):
                    base = (sy * src_w + x0) * src_bpp
                    for sx in range(xw):
                        p = base + sx * src_bpp
                        if src_fmt == 1:
                            r = get_byte(src, p + 0)
                            g = get_byte(src, p + 1)
                            b = get_byte(src, p + 2)
                        else:
                            b = get_byte(src, p + 0)
                            g = get_byte(src, p + 1)
                            r = get_byte(src, p + 2)
                        sr += r
                        sg += g
                        sb += b
                half = n // 2
                r = (sr + half) // n
                g = (sg + half) // n
                b = (sb + half) // n
                if dst_fmt == 2:
                    out[oy * dst_w + ox] = ((77 * r + 150 * g + 29 * b) >> 8) & 0xFF
                else:
                    d = (oy * dst_w + ox) * 3
                    out[d + 0] = r & 0xFF
                    out[d + 1] = g & 0xFF
                    out[d + 2] = b & 0xFF
        return out

    def run_transfer():
        op = regs[1]
        src_ptr = regs[2]
        dst_ptr = regs[3]
        src_w = regs[4]
        src_h = regs[5]
        dst_w = regs[6]
        dst_h = regs[7]
        src_fmt = regs[8]
        dst_fmt = regs[9]
        if src_ptr == 0 or dst_ptr == 0:
            return 0xFFFFFFFE
        if src_w == 0 or src_h == 0 or dst_w == 0 or dst_h == 0:
            return 0xFFFFFFFD
        if dst_w > src_w or dst_h > src_h:
            return 0xFFFFFFFC
        if src_fmt not in (0, 1) or dst_fmt not in (1, 2):
            return 0xFFFFFFFB
        if op not in (1, 2):
            return 0xFFFFFFFA
        src_bpp = 3 if src_fmt == 1 else 4
        src_len = src_w * src_h * src_bpp
        src = read_guest(src_ptr, src_len)
        out = area_scale_rgb(src, src_w, src_h, dst_w, dst_h, src_fmt, dst_fmt)
        write_guest(dst_ptr, out)
        regs[14] = len(out) & 0xFFFFFFFF
        return 0

    log_line("pxp accel ready")

elif request.IsRead:
    idx = request.Offset // 4
    request.Value = regs[idx] if idx < len(regs) else 0

elif request.IsWrite:
    idx = request.Offset // 4
    if idx < len(regs):
        regs[idx] = request.Value & 0xFFFFFFFF

    if request.Offset == 0x00 and request.Value == 1:
        regs[10] = 0
        regs[11] = 0xFFFFFFFF
        start = now_ms()
        try:
            rc = run_transfer()
            elapsed = now_ms() - start
            regs[11] = rc & 0xFFFFFFFF
            regs[12] = elapsed & 0xFFFFFFFF
            regs[13] = (regs[13] + 1) & 0xFFFFFFFF
            regs[15] = (regs[15] + elapsed) & 0xFFFFFFFF
            regs[16] = max(regs[16], elapsed) & 0xFFFFFFFF
            regs[10] = 2
            if regs[13] == 1 or (regs[13] % 25) == 0:
                log_line("transfer count=" + str(regs[13]) +
                         " rc=" + str(rc) + " elapsed_ms=" +
                         str(elapsed) + " src=" + str(regs[4]) +
                         "x" + str(regs[5]) + " dst=" +
                         str(regs[6]) + "x" + str(regs[7]) +
                         " sfmt=" + str(regs[8]) +
                         " dfmt=" + str(regs[9]))
        except Exception as e:
            regs[11] = 0xFFFFFFFF
            regs[17] = (regs[17] + 1) & 0xFFFFFFFF
            regs[10] = 2
            log_line("exception " + str(e))
