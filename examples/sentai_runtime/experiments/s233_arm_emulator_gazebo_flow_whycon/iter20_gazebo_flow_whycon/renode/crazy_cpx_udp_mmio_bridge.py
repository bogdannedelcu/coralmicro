import System
import socket
from System import Array, Byte
from System.Diagnostics import Stopwatch
from System.IO import File
from System.Threading import Thread

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 32
    bridge_sw = Stopwatch.StartNew()
    log_path = "/tmp/sentai_emu_crazy_cpx_udp_bridge.log"
    udp_host = "127.0.0.1"
    udp_port = 19850
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    udp.connect((udp_host, udp_port))

    cpx_start = 0xFF
    cpx_mtu = 100
    cpx_cts = [0xFF, 0x00]
    cpx_t_stm32 = 1
    cpx_t_host = 3
    cpx_f_system = 1
    cpx_f_console = 2
    cpx_f_crtp = 3
    crtp_max_datagram = 31
    rx_queue = []
    serial_buf = []
    opened = [False]

    try:
        if File.Exists(log_path):
            File.Delete(log_path)
    except Exception:
        pass

    def now_ms():
        try:
            return int(bridge_sw.ElapsedMilliseconds)
        except Exception:
            return 0

    def log_line(text):
        try:
            File.AppendAllText(log_path, "t=" + str(now_ms()) + "ms " + text + "\n")
        except Exception:
            pass

    def to_array(values):
        arr = Array.CreateInstance(Byte, len(values))
        for i in range(len(values)):
            arr[i] = values[i] & 0xFF
        return arr

    def from_array(arr, length):
        out = []
        for i in range(length):
            out.append(int(arr[i]) & 0xFF)
        return out

    def to_bytes(values):
        return bytes(bytearray([v & 0xFF for v in values]))

    def from_bytes(data):
        out = []
        for b in data:
            out.append(ord(b) if isinstance(b, str) else int(b))
        return out

    def cpx_crc(values):
        crc = 0
        for b in values:
            crc = (crc ^ (int(b) & 0xFF)) & 0xFF
        return crc

    def pack_route(dst, src, fn):
        return [
            (1 << 6) | ((src & 0x07) << 3) | (dst & 0x07),
            fn & 0x3F,
        ]

    def build_cpx_frame(data):
        frame = [cpx_start, len(data)] + data
        frame.append(cpx_crc(frame))
        return frame

    def enqueue(values):
        for b in values:
            rx_queue.append(b & 0xFF)

    def send_udp(values):
        return udp.send(to_bytes(values))

    def poll_udp():
        while True:
            try:
                data = udp.recv(256)
            except Exception:
                return
            if len(data) <= 0:
                return
            data = from_bytes(data)
            regs[13] = (regs[13] + 1) & 0xFFFFFFFF
            regs[15] = (regs[15] + n) & 0xFFFFFFFF
            if len(data) <= crtp_max_datagram:
                frame = build_cpx_frame(
                    pack_route(cpx_t_host, cpx_t_stm32, cpx_f_crtp) + data)
                enqueue(frame)
                log_line("udp->guest crtp_len=" + str(len(data)))
            else:
                regs[10] = (regs[10] + 1) & 0xFFFFFFFF
                log_line("drop oversized udp crtp len=" + str(len(data)))

    def handle_cpx_data(data):
        if len(data) < 2:
            return
        dst = data[0] & 0x07
        src = (data[0] >> 3) & 0x07
        fn = data[1] & 0x3F
        payload = data[2:]
        regs[18] = (regs[18] + 1) & 0xFFFFFFFF
        enqueue(cpx_cts)
        if fn == cpx_f_crtp:
            send_udp(payload)
            regs[12] = (regs[12] + 1) & 0xFFFFFFFF
            regs[14] = (regs[14] + len(payload)) & 0xFFFFFFFF
            log_line("guest->udp crtp_len=" + str(len(payload)))
        elif fn == cpx_f_system:
            regs[16] = (regs[16] + 1) & 0xFFFFFFFF
            log_line("guest system dst=" + str(dst) + " src=" + str(src) +
                     " payload=" + "".join(["%02x" % b for b in payload]))
        elif fn == cpx_f_console:
            regs[17] = (regs[17] + 1) & 0xFFFFFFFF
            log_line("guest console len=" + str(len(payload)))
        else:
            log_line("ignored cpx fn=" + str(fn) + " len=" + str(len(payload)))

    def parse_serial():
        while True:
            try:
                start = serial_buf.index(cpx_start)
            except Exception:
                del serial_buf[:]
                return
            if start > 0:
                del serial_buf[:start]
            if len(serial_buf) < 2:
                return
            cpx_len = serial_buf[1]
            if cpx_len == 0:
                regs[9] = (regs[9] + 1) & 0xFFFFFFFF
                del serial_buf[:2]
                continue
            if cpx_len > cpx_mtu:
                regs[10] = (regs[10] + 1) & 0xFFFFFFFF
                del serial_buf[:1]
                continue
            total = 2 + cpx_len + 1
            if len(serial_buf) < total:
                return
            frame = serial_buf[:total]
            expected = cpx_crc(frame[:-1])
            got = frame[-1]
            if got != expected:
                regs[10] = (regs[10] + 1) & 0xFFFFFFFF
                log_line("bad crc got=" + str(got) + " expected=" + str(expected))
                del serial_buf[:1]
                continue
            data = frame[2:2 + cpx_len]
            del serial_buf[:total]
            handle_cpx_data(data)

    def set_done(result):
        regs[6] = result & 0xFFFFFFFF
        regs[0] = 2

    def set_error(result):
        regs[6] = result & 0xFFFFFFFF
        regs[0] = 3

elif request.IsRead:
    idx = request.Offset // 4
    request.Value = regs[idx] if idx < len(regs) else 0

elif request.IsWrite:
    idx = request.Offset // 4
    if idx < len(regs):
        regs[idx] = request.Value & 0xFFFFFFFF

    if request.Offset == 0x00 and request.Value == 1:
        command = regs[1]
        data_ptr = regs[2]
        data_len = regs[3]
        out_ptr = regs[4]
        out_len = regs[5]
        try:
            if command == 1:
                opened[0] = True
                enqueue(cpx_cts)
                log_line("open")
                set_done(1)
            elif command == 2:
                opened[0] = False
                del rx_queue[:]
                del serial_buf[:]
                log_line("close")
                set_done(0)
            elif command == 3:
                data = []
                if data_len > 0:
                    arr = sysbus.ReadBytes(data_ptr, data_len)
                    data = from_array(arr, data_len)
                for b in data:
                    serial_buf.append(b)
                regs[11] = (regs[11] + data_len) & 0xFFFFFFFF
                parse_serial()
                poll_udp()
                set_done(data_len)
            elif command == 4:
                poll_udp()
                n = out_len if out_len < len(rx_queue) else len(rx_queue)
                if n > 0 and out_ptr != 0:
                    sysbus.WriteBytes(to_array(rx_queue[:n]), out_ptr)
                    del rx_queue[:n]
                    regs[15] = (regs[15] + n) & 0xFFFFFFFF
                set_done(n)
            elif command == 5:
                poll_udp()
                set_done(len(rx_queue))
            elif command == 6:
                log_line("baudrate " + str(regs[8]))
                set_done(0)
            else:
                set_error(0xFFFFFFFE)
        except Exception as e:
            log_line("exception " + str(e))
            set_error(0xFFFFFFFF)
