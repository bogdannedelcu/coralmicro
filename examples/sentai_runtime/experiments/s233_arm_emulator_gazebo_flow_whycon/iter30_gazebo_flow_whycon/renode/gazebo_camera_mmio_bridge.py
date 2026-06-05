import System
import clr
import socket
import struct
from System import Array, Byte, UInt64
from System.Diagnostics import Stopwatch
from System.IO import File
from System.Text import Encoding

clr.AddReference("System")

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 32
    sw = Stopwatch.StartNew()
    log_path = "/tmp/sentai_emu_gazebo_camera_bridge.log"
    host = "127.0.0.1"
    port = 30233
    scm_magic = 0x53434D31
    get_magic = 0x47455431
    expect_w = 640
    expect_h = 480
    expect_rgb_bytes = expect_w * expect_h * 3
    expect_xrgb_bytes = expect_w * expect_h * 4
    served_seq = [0]
    pull_count = [0]
    empty_count = [0]
    bad_count = [0]
    tcp = [None]
    stream = [None]
    latin1 = Encoding.GetEncoding("ISO-8859-1")

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

    def to_array(data):
        if isinstance(data, str):
            return latin1.GetBytes(data)
        try:
            return Array[Byte](data)
        except Exception:
            pass
        arr = Array.CreateInstance(Byte, len(data))
        for i in range(len(data)):
            v = data[i]
            try:
                iv = int(v)
            except Exception:
                iv = ord(v)
            arr[i] = iv & 0xFF
        return arr

    def recv_full(conn, n):
        out = bytearray()
        while len(out) < n:
            chunk = conn.recv(n - len(out))
            if not chunk:
                return None
            out.extend(chunk)
        return bytes(out)

    def close_tcp():
        try:
            if tcp[0] is not None:
                tcp[0].Close()
        except Exception:
            pass
        tcp[0] = None
        stream[0] = None

    def ensure_tcp():
        if tcp[0] is not None:
            return stream[0]
        c = System.Net.Sockets.TcpClient()
        c.Connect(host, port)
        c.ReceiveTimeout = 1000
        c.SendTimeout = 1000
        tcp[0] = c
        stream[0] = c.GetStream()
        log_line("connected to host pull service")
        return stream[0]

    def write_all(stream_obj, arr):
        stream_obj.Write(arr, 0, len(arr))

    def recv_array(stream_obj, n):
        arr = Array.CreateInstance(Byte, n)
        off = 0
        while off < n:
            got = stream_obj.Read(arr, off, n - off)
            if got <= 0:
                return None
            off += got
        return arr

    def array_to_bytes(arr, n):
        out = bytearray(n)
        for i in range(n):
            out[i] = int(arr[i]) & 0xFF
        return bytes(out)

    def pull_latest():
        try:
            conn = ensure_tcp()
            write_all(conn, to_array(struct.pack("<II", get_magic,
                                                 served_seq[0])))
            hdr_arr = recv_array(conn, 24)
            if hdr_arr is None:
                close_tcp()
                return None
            hdr = array_to_bytes(hdr_arr, 24)
            magic, seq, w, h, fmt, nbytes = struct.unpack("<IIIIII", hdr)
            if magic != scm_magic:
                bad_count[0] += 1
                log_line("bad magic 0x" + format(magic, "08x"))
                close_tcp()
                return None
            if seq == 0 or nbytes == 0:
                empty_count[0] += 1
                return None
            valid_frame = (
                w == expect_w and h == expect_h and
                ((fmt == 0 and nbytes == expect_rgb_bytes) or
                 (fmt == 1 and nbytes == expect_xrgb_bytes)))
            if not valid_frame:
                bad_count[0] += 1
                log_line("bad frame seq=" + str(seq) +
                         " w=" + str(w) + " h=" + str(h) +
                         " fmt=" + str(fmt) + " bytes=" + str(nbytes))
                close_tcp()
                return None
            data = recv_array(conn, nbytes)
            if data is None:
                close_tcp()
                return None
            pull_count[0] += 1
            if pull_count[0] == 1 or (pull_count[0] % 5) == 0:
                log_line("pulled seq=" + str(seq) +
                         " pulls=" + str(pull_count[0]) +
                         " empty=" + str(empty_count[0]))
            return seq, data, nbytes, fmt
        except Exception as e:
            bad_count[0] += 1
            log_line("pull exception " + str(e))
            close_tcp()
            return None

    log_line("pull peripheral ready " + host + ":" + str(port))

elif request.IsRead:
    idx = request.Offset // 4
    request.Value = regs[idx] if idx < len(regs) else 0

elif request.IsWrite:
    idx = request.Offset // 4
    if idx < len(regs):
        regs[idx] = request.Value & 0xFFFFFFFF

    if request.Offset == 0x00 and request.Value == 1:
        command = regs[1]
        out_ptr = regs[2]
        out_len = regs[3]
        regs[4] = 0xFFFFFFFF
        try:
            if command == 1:
                expected_fmt = regs[13] & 0xFFFFFFFF
                pulled = pull_latest()
                if pulled is None:
                    regs[4] = 0
                    regs[5] = 0
                    regs[6] = 0
                    regs[14] = 0xFFFFFFFF
                    regs[15] = 0
                else:
                    seq, data, nbytes, actual_fmt = pulled
                    if out_ptr == 0 or out_len < nbytes or \
                            expected_fmt != actual_fmt:
                        regs[4] = 0xFFFFFFFE
                        regs[14] = int(actual_fmt) & 0xFFFFFFFF
                        regs[15] = int(nbytes) & 0xFFFFFFFF
                    else:
                        log_line("write begin seq=" + str(seq) +
                                 " bytes=" + str(nbytes))
                        sysbus.WriteBytes(data, UInt64.Parse(str(out_ptr)))
                        log_line("write done seq=" + str(seq))
                        served_seq[0] = int(seq)
                        regs[4] = nbytes
                        regs[5] = int(seq) & 0xFFFFFFFF
                        regs[6] = expect_w
                        regs[7] = expect_h
                        regs[8] = int(seq) & 0xFFFFFFFF
                        regs[9] = pull_count[0] & 0xFFFFFFFF
                        regs[10] = empty_count[0] & 0xFFFFFFFF
                        regs[11] = bad_count[0] & 0xFFFFFFFF
                        regs[12] = pull_count[0] & 0xFFFFFFFF
                        regs[14] = int(actual_fmt) & 0xFFFFFFFF
                        regs[15] = int(nbytes) & 0xFFFFFFFF
            elif command == 2:
                regs[4] = served_seq[0] & 0xFFFFFFFF
                regs[5] = pull_count[0] & 0xFFFFFFFF
            else:
                regs[4] = 0xFFFFFFFD
        except Exception as e:
            log_line("mmio exception " + str(e) +
                     " out_ptr=" + str(out_ptr) +
                     " out_len=" + str(out_len))
            regs[4] = 0xFFFFFFFF
        regs[0] = 2
