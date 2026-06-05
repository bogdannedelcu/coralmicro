import System
import socket
import struct
from System import Array, Byte, UInt64
from System.Diagnostics import Stopwatch
from System.IO import File
from System.Text import Encoding
from System.Threading import Thread, ThreadStart

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 32
    sw = Stopwatch.StartNew()
    log_path = "/tmp/sentai_emu_gazebo_camera_bridge.log"
    listen_host = "127.0.0.1"
    listen_port = 30233
    magic = 0x53434D31
    reply_magic = 0x46524C31
    expect_w = 640
    expect_h = 480
    expect_bytes = expect_w * expect_h * 3
    latest = {"seq": 0, "data": None}
    latin1 = Encoding.GetEncoding("ISO-8859-1")
    served_seq = [0]
    dropped = [0]
    bad = [0]
    running = [True]

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

    def send_dummy_reply(conn, seq):
        payload = struct.pack(
            "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x",
            reply_magic, seq, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0)
        conn.sendall(payload)

    def accept_loop():
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except Exception:
                pass
            srv.bind((listen_host, listen_port))
            srv.listen(1)
            log_line("listening " + listen_host + ":" + str(listen_port))
            while running[0]:
                conn, _ = srv.accept()
                log_line("client connected")
                try:
                    while running[0]:
                        hdr = recv_full(conn, 24)
                        if hdr is None:
                            break
                        m, seq, w, h, fmt, nbytes = struct.unpack("<IIIIII", hdr)
                        if m != magic or w != expect_w or h != expect_h or \
                           fmt != 0 or nbytes != expect_bytes:
                            bad[0] += 1
                            log_line("bad header seq=" + str(seq) +
                                     " w=" + str(w) + " h=" + str(h) +
                                     " fmt=" + str(fmt) + " bytes=" +
                                     str(nbytes))
                            break
                        data = recv_full(conn, nbytes)
                        if data is None:
                            break
                        if latest["data"] is not None and latest["seq"] != served_seq[0]:
                            dropped[0] += 1
                        latest["seq"] = seq
                        latest["data"] = data
                        regs[8] = seq & 0xFFFFFFFF
                        regs[9] = (regs[9] + 1) & 0xFFFFFFFF
                        regs[10] = dropped[0] & 0xFFFFFFFF
                        regs[11] = bad[0] & 0xFFFFFFFF
                        if (seq % 30) == 0:
                            log_line("frame seq=" + str(seq) +
                                     " dropped=" + str(dropped[0]))
                        send_dummy_reply(conn, seq)
                except Exception as e:
                    log_line("client exception " + str(e))
                try:
                    conn.close()
                except Exception:
                    pass
                log_line("client disconnected")
        except Exception as e:
            log_line("server exception " + str(e))

    t = Thread(ThreadStart(accept_loop))
    t.IsBackground = True
    t.Start()

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
                data = latest["data"]
                seq = int(latest["seq"])
                if data is None or seq == served_seq[0]:
                    regs[4] = 0
                    regs[5] = 0
                    regs[6] = 0
                elif out_ptr == 0 or out_len < len(data):
                    regs[4] = 0xFFFFFFFE
                else:
                    sysbus.WriteBytes(to_array(data),
                                      UInt64.Parse(str(out_ptr)))
                    served_seq[0] = seq
                    regs[4] = len(data)
                    regs[5] = seq & 0xFFFFFFFF
                    regs[6] = expect_w
                    regs[7] = expect_h
                    regs[12] = (regs[12] + 1) & 0xFFFFFFFF
            elif command == 2:
                regs[4] = int(latest["seq"]) & 0xFFFFFFFF
                regs[5] = served_seq[0] & 0xFFFFFFFF
            else:
                regs[4] = 0xFFFFFFFD
        except Exception as e:
            log_line("mmio exception " + str(e) +
                     " out_ptr=" + str(out_ptr) +
                     " out_len=" + str(out_len))
            regs[4] = 0xFFFFFFFF
        regs[0] = 2
