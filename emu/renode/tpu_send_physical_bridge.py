import System
import os
import signal
import subprocess
from System.Diagnostics import Stopwatch
from System.IO import File, FileMode, FileAccess, Path
from System.Threading import Thread, ThreadStart

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 32
    server_cmd_path = "/tmp/sentai_emu_tpu_send_server.cmd"
    server_path = "/home/bogdan/work/coralmicro/build-sim/sim/tpu_posix_send_server"
    log_path = "/tmp/sentai_emu_tpu_send_bridge.log"
    bridge_sw = Stopwatch.StartNew()
    server_proc = [None]
    server_reader_thread = [None]
    server_lines = []
    server_read_idx = [0]
    server_cmd_seq = [0]
    payload_dir = "/tmp/sentai_emu_tpu_send_payloads"
    server_perf = os.environ.get("SENTAI_TPU_SEND_PERF", "low")
    server_chunk_size = os.environ.get(
        "SENTAI_TPU_SEND_CHUNK_SIZE", str(1024 * 1024))
    server_bulkin_chunk_size = os.environ.get(
        "SENTAI_TPU_SEND_BULKIN_CHUNK_SIZE", "1024")
    server_outfeed_chunk_length = os.environ.get(
        "SENTAI_TPU_SEND_OUTFEED_CHUNK_LENGTH", "0x80")
    server_bulkin_queue_depth = os.environ.get(
        "SENTAI_TPU_SEND_BULKIN_QUEUE_DEPTH", "")

    try:
        subprocess.call(["pkill", "-f", server_path])
        if System.IO.File.Exists(log_path):
            System.IO.File.Delete(log_path)
        if System.IO.File.Exists(server_cmd_path):
            System.IO.File.Delete(server_cmd_path)
        if not System.IO.Directory.Exists(payload_dir):
            System.IO.Directory.CreateDirectory(payload_dir)
    except Exception:
        pass

    def now_ms():
        try:
            return int(bridge_sw.ElapsedMilliseconds)
        except Exception:
            return 0

    def log_line(text):
        System.IO.File.AppendAllText(
            log_path, "t=" + str(now_ms()) + "ms " + text + "\n")

    def as_text(data):
        if data is None:
            return ""
        if hasattr(data, "decode"):
            return data.decode("utf-8", "replace")
        return str(data)

    def text_has_marker(text, markers):
        for line in text.splitlines():
            for marker in markers:
                if line.startswith(marker):
                    return True
        return False

    def text_has_done_rc0(text):
        for line in text.splitlines():
            if line.startswith("SEND_SERVER_DONE ") and " rc=0 " in line:
                return True
        return False

    def server_reader_loop():
        proc = server_proc[0]
        if proc is None:
            return
        while True:
            try:
                raw = proc.stdout.readline()
            except Exception as e:
                log_line("server reader exception " + str(e))
                break
            line = as_text(raw)
            if raw is None or line == "":
                break
            line = line.rstrip("\r\n")
            server_lines.append(line)
            log_line("server " + line)

    def read_server_until(markers, timeout_ms):
        proc = server_proc[0]
        lines = []
        if proc is None:
            return ""
        deadline = now_ms() + timeout_ms
        while True:
            while server_read_idx[0] < len(server_lines):
                line = server_lines[server_read_idx[0]]
                server_read_idx[0] = server_read_idx[0] + 1
                lines.append(line)
                for marker in markers:
                    if line.startswith(marker):
                        return "\n".join(lines)
            if proc.poll() is not None:
                break
            if timeout_ms > 0 and now_ms() >= deadline:
                log_line("server read timeout markers=" + ",".join(markers))
                break
            Thread.Sleep(5)
        return "\n".join(lines)

    def terminate_server():
        proc = server_proc[0]
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.terminate()
        except Exception:
            pass
        try:
            if proc.poll() is None:
                os.kill(proc.pid, signal.SIGTERM)
        except Exception:
            pass
        for _ in range(50):
            try:
                if proc.poll() is not None:
                    break
            except Exception:
                break
            Thread.Sleep(10)
        try:
            if proc.poll() is None:
                proc.kill()
        except Exception:
            pass
        try:
            proc.wait()
        except Exception:
            pass
        try:
            proc.stdout.close()
        except Exception:
            pass
        try:
            proc.stderr.close()
        except Exception:
            pass
        try:
            reader = server_reader_thread[0]
            if reader is not None:
                reader.Join(1000)
        except Exception:
            pass
        server_reader_thread[0] = None
        server_proc[0] = None

    def stop_server():
        proc = server_proc[0]
        if proc is None:
            return
        try:
            if proc.poll() is None:
                server_cmd_seq[0] = server_cmd_seq[0] + 1
                System.IO.File.WriteAllText(
                    server_cmd_path, str(server_cmd_seq[0]) + " stop\n")
                read_server_until(("SEND_SERVER_STOPPED",), 2000)
        except Exception as e:
            log_line("server stop exception " + str(e))
        terminate_server()

    def server_alive():
        proc = server_proc[0]
        return proc is not None and proc.poll() is None

    def start_server():
        if server_alive():
            return True
        stop_server()
        try:
            if System.IO.File.Exists(server_cmd_path):
                System.IO.File.Delete(server_cmd_path)
        except Exception:
            pass
        server_cmd_seq[0] = 0
        argv = [
            server_path,
            "--server-cmd",
            server_cmd_path,
            "--perf",
            server_perf,
            "--chunk-size",
            server_chunk_size,
            "--bulkin-chunk-size",
            server_bulkin_chunk_size,
            "--outfeed-chunk-length",
            server_outfeed_chunk_length,
        ]
        if server_bulkin_queue_depth:
            argv.extend(["--bulkin-queue-depth", server_bulkin_queue_depth])
        log_line("server start argv=" + " ".join(argv))
        proc = subprocess.Popen(
            argv,
            cwd="/home/bogdan/work/coralmicro",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        server_proc[0] = proc
        server_lines[:] = []
        server_read_idx[0] = 0
        reader = Thread(ThreadStart(server_reader_loop))
        reader.IsBackground = True
        server_reader_thread[0] = reader
        reader.Start()
        text = read_server_until(("SEND_SERVER_READY", "SEND_SERVER_FAIL"), 20000)
        ok = text_has_marker(text, ("SEND_SERVER_READY",)) and proc.poll() is None
        if not ok:
            stop_server()
        return ok

    def server_send(command, timeout_ms):
        if not start_server():
            return False, "SEND_SERVER_NOT_RUNNING"
        log_line("server command " + command)
        server_cmd_seq[0] = server_cmd_seq[0] + 1
        System.IO.File.WriteAllText(
            server_cmd_path, str(server_cmd_seq[0]) + " " + command + "\n")
        text = read_server_until(("SEND_SERVER_DONE ", "SEND_SERVER_FAIL"), timeout_ms)
        ok = server_alive() and text_has_done_rc0(text)
        if not ok:
            log_line("server command timeout/fail command=" + command)
            terminate_server()
        return ok, text

    def write_bytes(path, data):
        fs = File.Open(path, FileMode.Create, FileAccess.Write)
        fs.Write(data, 0, len(data))
        fs.Close()

    def read_bytes(path):
        fs = File.Open(path, FileMode.Open, FileAccess.Read)
        n = int(fs.Length)
        arr = System.Array.CreateInstance(System.Byte, n)
        got = fs.Read(arr, 0, n)
        fs.Close()
        if got != n:
            raise Exception("short host output read")
        return arr

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
        seq = regs[7]
        regs[6] = 0xFFFFFFFF
        try:
            op_name = "unknown"
            ok = False
            text = ""
            t0 = now_ms()
            if command in (1, 2, 3):
                op_name = "params" if command == 1 else ("inputs" if command == 2 else "ins")
                payload_path = Path.Combine(
                    payload_dir, "payload_" + str(seq) + "_" + op_name + ".bin")
                data = System.Array.CreateInstance(System.Byte, data_len)
                if data_len > 0:
                    data = sysbus.ReadBytes(data_ptr, data_len)
                write_bytes(payload_path, data)
                ok, text = server_send(op_name + " " + payload_path, 120000)
            elif command == 4:
                op_name = "output"
                output_path = Path.Combine(payload_dir, "output_" + str(seq) + ".bin")
                ok, text = server_send(
                    "output " + str(out_len) + " " + output_path, 120000)
                if ok and out_ptr != 0 and out_len != 0:
                    out = read_bytes(output_path)
                    sysbus.WriteBytes(out, out_ptr)
            elif command == 5:
                op_name = "event"
                ok, text = server_send("event", 120000)
            regs[6] = 0 if ok else 1
            regs[8] = data_len & 0xFFFFFFFF
            regs[9] = out_len & 0xFFFFFFFF
            regs[10] = (now_ms() - t0) & 0xFFFFFFFF
            regs[11] = (regs[11] + 1) & 0xFFFFFFFF
            log_line(
                "bridge cmd=" + op_name +
                " seq=" + str(seq) +
                " data_len=" + str(data_len) +
                " out_len=" + str(out_len) +
                " ok=" + str(ok) +
                " ms=" + str(regs[10]))
            regs[0] = 2 if ok else 3
        except Exception as e:
            try:
                log_line("exception " + str(e))
            except Exception:
                pass
            regs[6] = 1
            regs[0] = 3
