import System
import os
import signal
import subprocess
from System.Diagnostics import Stopwatch
from System.IO import File, FileMode, FileAccess
from System.Threading import Thread, ThreadStart

if request.IsInit:
    sysbus = emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus
    regs = [0] * 48
    model_path = "/tmp/sentai_emu_tpu_model.tflite"
    image_path = "/tmp/sentai_emu_tpu_image.bmp"
    server_cmd_path = "/tmp/sentai_emu_tpu_server.cmd"
    smoke_path = "/home/bogdan/work/coralmicro/build-sim/sim/tpu_posix_invoke_smoke"
    detections = []
    log_path = "/tmp/sentai_emu_tpu_bridge.log"
    bridge_sw = Stopwatch.StartNew()
    server_proc = [None]
    server_reader_thread = [None]
    server_lines = []
    server_read_idx = [0]
    file_start_ms = [0] * 3
    file_chunks = [0] * 3
    file_guest_read_ms = [0] * 3
    file_host_write_ms = [0] * 3
    server_cmd_seq = [0]
    try:
        if System.IO.File.Exists(log_path):
            System.IO.File.Delete(log_path)
        if System.IO.File.Exists(server_cmd_path):
            System.IO.File.Delete(server_cmd_path)
    except Exception:
        pass

    def set_status(ok):
        regs[5] = 1 if ok else 2

    def path_for(kind):
        if kind == 1:
            return model_path
        if kind == 2:
            return image_path
        return None

    def parse_detections(text):
        out = []
        for line in text.splitlines():
            if not line.startswith("DET[") or " class=" not in line or " score=" not in line or " box=[" not in line:
                continue
            try:
                cls_s = line.split(" class=", 1)[1].split(" ", 1)[0]
                score_s = line.split(" score=", 1)[1].split(" ", 1)[0]
                box_s = line.split(" box=[", 1)[1].split("]", 1)[0]
                box = box_s.split(" ")
                if len(box) != 4:
                    continue
                cls = int(cls_s)
                score = int(round(float(score_s) * 1000.0))
                y0 = int(round(float(box[0]) * 1000.0))
                x0 = int(round(float(box[1]) * 1000.0))
                y1 = int(round(float(box[2]) * 1000.0))
                x1 = int(round(float(box[3]) * 1000.0))
            except Exception:
                continue
            out.append((cls, score, y0, x0, y1, x1))
            if len(out) >= 20:
                break
        return out

    def parse_detection_count(text):
        marker = "DETECTIONS n="
        pos = text.find(marker)
        if pos < 0:
            return 0
        pos += len(marker)
        end = pos
        while end < len(text) and text[end].isdigit():
            end += 1
        if end == pos:
            return 0
        try:
            return int(text[pos:end])
        except Exception:
            return 0

    def parse_invoke_ms(text):
        marker = "invoke_ms="
        pos = text.find(marker)
        if pos < 0:
            return 0
        pos += len(marker)
        end = pos
        while end < len(text) and text[end].isdigit():
            end += 1
        if end == pos:
            return 0
        return int(text[pos:end])

    def parse_kv_int(text, key):
        marker = key + "="
        pos = text.find(marker)
        if pos < 0:
            return 0
        pos += len(marker)
        end = pos
        while end < len(text) and (text[end].isdigit() or text[end] == "-"):
            end += 1
        if end == pos:
            return 0
        try:
            return int(text[pos:end])
        except Exception:
            return 0

    def line_with_prefix(text, prefix):
        for line in text.splitlines():
            if line.startswith(prefix):
                return line
        return ""

    def run_smoke(extra_args):
        argv = [smoke_path] + extra_args + [model_path, image_path]
        proc = subprocess.Popen(
            argv,
            cwd="/home/bogdan/work/coralmicro",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        stdout_b, stderr_b = proc.communicate()
        rc = proc.wait()
        stdout = stdout_b.decode("utf-8", "replace") if hasattr(stdout_b, "decode") else str(stdout_b)
        stderr = stderr_b.decode("utf-8", "replace") if hasattr(stderr_b, "decode") else str(stderr_b)
        return rc, stdout, stderr

    def as_text(data):
        if data is None:
            return ""
        if hasattr(data, "decode"):
            return data.decode("utf-8", "replace")
        return str(data)

    def now_ms():
        try:
            return int(bridge_sw.ElapsedMilliseconds)
        except Exception:
            return 0

    def log_line(text):
        System.IO.File.AppendAllText(
            log_path, "t=" + str(now_ms()) + "ms " + text + "\n")

    def log_smoke_summary(text):
        prefixes = (
            "TPU_BENCH_CONFIG ",
            "OK tpu_posix_invoke ",
            "FAIL invoke ",
            "FPS_BENCH ",
            "TPU_STAGE_STATS ",
            "TPU_CALL_STATS ",
            "TPU_USB_STATS ",
            "TPU_DESC_CACHE_STATS ",
        )
        for line in text.splitlines():
            if line.startswith(prefixes):
                log_line("smoke " + line)

    def populate_tpu_stats(text):
        stage = line_with_prefix(text, "TPU_STAGE_STATS ")
        usb = line_with_prefix(text, "TPU_USB_STATS ")
        # Register map exposed to the guest by sentai.tpu.stats():
        # 16..29: EdgeTpuExecutable/TpuDriver stage counters.
        regs[16] = parse_kv_int(stage, "params_calls") & 0xFFFFFFFF
        regs[17] = parse_kv_int(stage, "params_bytes") & 0xFFFFFFFF
        regs[18] = parse_kv_int(stage, "params_ticks") & 0xFFFFFFFF
        regs[19] = parse_kv_int(stage, "input_calls") & 0xFFFFFFFF
        regs[20] = parse_kv_int(stage, "input_bytes") & 0xFFFFFFFF
        regs[21] = parse_kv_int(stage, "input_ticks") & 0xFFFFFFFF
        regs[22] = parse_kv_int(stage, "ins_calls") & 0xFFFFFFFF
        regs[23] = parse_kv_int(stage, "ins_bytes") & 0xFFFFFFFF
        regs[24] = parse_kv_int(stage, "ins_ticks") & 0xFFFFFFFF
        regs[25] = parse_kv_int(stage, "output_calls") & 0xFFFFFFFF
        regs[26] = parse_kv_int(stage, "output_bytes") & 0xFFFFFFFF
        regs[27] = parse_kv_int(stage, "output_ticks") & 0xFFFFFFFF
        regs[28] = parse_kv_int(stage, "event_calls") & 0xFFFFFFFF
        regs[29] = parse_kv_int(stage, "event_ticks") & 0xFFFFFFFF

        # 30..43: POSIX/libusb aggregate transfer counters for the same
        # measured window.
        regs[30] = parse_kv_int(usb, "out_calls") & 0xFFFFFFFF
        regs[31] = parse_kv_int(usb, "out_req") & 0xFFFFFFFF
        regs[32] = parse_kv_int(usb, "out_done") & 0xFFFFFFFF
        regs[33] = parse_kv_int(usb, "out_us") & 0xFFFFFFFF
        regs[34] = parse_kv_int(usb, "in_calls") & 0xFFFFFFFF
        regs[35] = parse_kv_int(usb, "in_req") & 0xFFFFFFFF
        regs[36] = parse_kv_int(usb, "in_done") & 0xFFFFFFFF
        regs[37] = parse_kv_int(usb, "in_us") & 0xFFFFFFFF
        regs[38] = parse_kv_int(usb, "event_calls") & 0xFFFFFFFF
        regs[39] = parse_kv_int(usb, "event_req") & 0xFFFFFFFF
        regs[40] = parse_kv_int(usb, "event_done") & 0xFFFFFFFF
        regs[41] = parse_kv_int(usb, "event_us") & 0xFFFFFFFF
        regs[42] = parse_kv_int(usb, "timeouts") & 0xFFFFFFFF
        regs[43] = parse_kv_int(usb, "failed") & 0xFFFFFFFF

    def server_alive():
        proc = server_proc[0]
        return proc is not None and proc.poll() is None

    def text_has_marker(text, markers):
        for line in text.splitlines():
            for marker in markers:
                if line.startswith(marker):
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
            if proc.poll() is None:
                os.kill(proc.pid, signal.SIGKILL)
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
            return "SERVER_STOPPED absent"
        text = ""
        try:
            if proc.poll() is None:
                server_cmd_seq[0] = server_cmd_seq[0] + 1
                System.IO.File.WriteAllText(
                    server_cmd_path, str(server_cmd_seq[0]) + " stop\n")
                text = read_server_until(("SERVER_STOPPED",), 2000)
        except Exception as e:
            text = "SERVER_STOP_EXCEPTION " + str(e)
            log_line(text)
        terminate_server()
        return text

    def start_server():
        if server_alive():
            stop_server()
        try:
            if System.IO.File.Exists(server_cmd_path):
                System.IO.File.Delete(server_cmd_path)
        except Exception:
            pass
        server_cmd_seq[0] = 0
        argv = [
            smoke_path,
            "--server",
            "--server-cmd",
            server_cmd_path,
            model_path,
            image_path,
        ]
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
        text = read_server_until(("SERVER_READY", "FAIL "), 15000)
        ok = text_has_marker(text, ("SERVER_READY",)) and proc.poll() is None
        if not ok:
            stop_server()
        return ok, text

    def server_send(command, markers):
        if not server_alive():
            return False, "SERVER_NOT_RUNNING"
        log_line("server command " + command)
        server_cmd_seq[0] = server_cmd_seq[0] + 1
        System.IO.File.WriteAllText(
            server_cmd_path, str(server_cmd_seq[0]) + " " + command + "\n")
        text = read_server_until(markers, 8000)
        ok = server_alive() and text_has_marker(text, markers)
        if not ok:
            log_line("server command timeout/fail command=" + command)
            terminate_server()
        return ok, text

elif request.IsRead:
    idx = request.Offset // 4
    request.Value = regs[idx] if idx < len(regs) else 0
elif request.IsWrite:
    idx = request.Offset // 4
    if idx < len(regs):
        regs[idx] = request.Value & 0xFFFFFFFF
    if request.Offset == 0x00:
        cmd = regs[0]
        kind = regs[1]
        expected_size = regs[2]
        guest_ptr = regs[3]
        length = regs[4]
        regs[5] = 0
        try:
            if cmd == 1:
                p = path_for(kind)
                if p is None:
                    set_status(False)
                else:
                    fs = File.Open(p, FileMode.Create, FileAccess.Write)
                    fs.Close()
                    regs[6] = 0
                    regs[9] = expected_size
                    if kind < len(file_start_ms):
                        file_start_ms[kind] = now_ms()
                        file_chunks[kind] = 0
                        file_guest_read_ms[kind] = 0
                        file_host_write_ms[kind] = 0
                    log_line(
                        "file begin kind=" + str(kind) +
                        " expected_size=" + str(expected_size) +
                        " path=" + p)
                    set_status(True)
            elif cmd == 2:
                p = path_for(kind)
                if p is None or guest_ptr == 0 or length == 0:
                    set_status(False)
                else:
                    read_t0 = now_ms()
                    data = sysbus.ReadBytes(guest_ptr, length)
                    read_t1 = now_ms()
                    write_t0 = now_ms()
                    fs = File.Open(p, FileMode.Append, FileAccess.Write)
                    fs.Write(data, 0, length)
                    fs.Close()
                    write_t1 = now_ms()
                    if kind < len(file_chunks):
                        file_chunks[kind] = file_chunks[kind] + 1
                        file_guest_read_ms[kind] = (
                            file_guest_read_ms[kind] + (read_t1 - read_t0))
                        file_host_write_ms[kind] = (
                            file_host_write_ms[kind] + (write_t1 - write_t0))
                    regs[6] = (regs[6] + length) & 0xFFFFFFFF
                    if regs[9] != 0 and regs[6] >= regs[9]:
                        total_ms = 0
                        chunks = 0
                        guest_read_ms = 0
                        host_write_ms = 0
                        if kind < len(file_start_ms):
                            total_ms = now_ms() - file_start_ms[kind]
                            chunks = file_chunks[kind]
                            guest_read_ms = file_guest_read_ms[kind]
                            host_write_ms = file_host_write_ms[kind]
                        log_line(
                            "file done kind=" + str(kind) +
                            " bytes=" + str(regs[6]) +
                            " chunks=" + str(chunks) +
                            " total_ms=" + str(total_ms) +
                            " guest_read_ms=" + str(guest_read_ms) +
                            " host_write_ms=" + str(host_write_ms))
                    set_status(True)
            elif cmd == 3:
                detections = []
                log_line("invoke begin")
                invoke_t0 = now_ms()
                rc, stdout, stderr = run_smoke([])
                invoke_host_ms = now_ms() - invoke_t0
                regs[9] = rc & 0xFFFFFFFF
                log_smoke_summary(stdout + "\n" + stderr)
                populate_tpu_stats(stdout + "\n" + stderr)
                if rc != 0:
                    log_line(
                        "invoke exit=" + str(rc) +
                        " host_ms=" + str(invoke_host_ms))
                    log_line(stderr)
                    set_status(False)
                else:
                    detections = parse_detections(stdout + "\n" + stderr)
                    count = parse_detection_count(stdout + "\n" + stderr)
                    if count <= 0:
                        count = len(detections)
                    regs[7] = count
                    regs[8] = parse_invoke_ms(stdout) & 0xFFFFFFFF
                    log_line(
                        "invoke ok detection_count=" + str(count) +
                        " parsed_detections=" + str(len(detections)) +
                        " host_ms=" + str(invoke_host_ms) +
                        " smoke_invoke_ms=" + str(regs[8])
                    )
                    set_status(count > 0 and len(detections) > 0)
            elif cmd == 6:
                ok, text = start_server()
                regs[9] = 0 if ok else 1
                set_status(ok)
            elif cmd == 7:
                stop_server()
                regs[9] = 0
                set_status(True)
            elif cmd == 8:
                detections = []
                invoke_t0 = now_ms()
                ok, text = server_send("invoke", ("SERVER_DONE ", "FAIL "))
                invoke_host_ms = now_ms() - invoke_t0
                log_smoke_summary(text)
                populate_tpu_stats(text)
                count = parse_detection_count(text)
                detections = parse_detections(text)
                if count <= 0:
                    count = len(detections)
                regs[7] = count
                regs[8] = parse_invoke_ms(text) & 0xFFFFFFFF
                regs[9] = parse_kv_int(text, "rc") & 0xFFFFFFFF
                log_line(
                    "session invoke ok=" + str(ok) +
                    " detection_count=" + str(count) +
                    " parsed_detections=" + str(len(detections)) +
                    " host_ms=" + str(invoke_host_ms) +
                    " smoke_invoke_ms=" + str(regs[8])
                )
                set_status(ok and regs[9] == 0 and count > 0 and len(detections) > 0)
            elif cmd == 9:
                detections = []
                runs = kind
                warmup = expected_size
                if runs <= 0:
                    runs = 1
                fps_t0 = now_ms()
                ok, text = server_send(
                    "fps " + str(runs) + " " + str(warmup),
                    ("SERVER_DONE ", "FAIL "))
                fps_host_ms = now_ms() - fps_t0
                log_smoke_summary(text)
                populate_tpu_stats(text)
                count = parse_detection_count(text)
                detections = parse_detections(text)
                if count <= 0:
                    count = len(detections)
                regs[7] = count
                regs[8] = parse_kv_int(text, "measured_ms") & 0xFFFFFFFF
                regs[9] = parse_kv_int(text, "rc") & 0xFFFFFFFF
                regs[10] = parse_kv_int(text, "completed") & 0xFFFFFFFF
                regs[11] = warmup & 0xFFFFFFFF
                regs[12] = parse_kv_int(text, "fps_x100") & 0xFFFFFFFF
                regs[13] = parse_kv_int(text, "invoke_ms_sum") & 0xFFFFFFFF
                log_line(
                    "session fps ok=" + str(ok) +
                    " completed_runs=" + str(regs[10]) +
                    " fps_x100=" + str(regs[12]) +
                    " detection_count=" + str(count) +
                    " parsed_detections=" + str(len(detections)) +
                    " host_ms=" + str(fps_host_ms) +
                    " measured_ms=" + str(regs[8]) +
                    " invoke_ms_sum=" + str(regs[13])
                )
                set_status(ok and regs[9] == 0 and regs[10] > 0 and count > 0 and len(detections) > 0)
            elif cmd == 10:
                ok, text = server_send("image", ("SERVER_IMAGE_OK", "SERVER_IMAGE_FAIL"))
                regs[9] = 0 if ok and "SERVER_IMAGE_OK" in text else 1
                set_status(regs[9] == 0)
            elif cmd == 5:
                detections = []
                runs = kind
                warmup = expected_size
                if runs <= 0:
                    runs = 1
                log_line("fps begin runs=" + str(runs) + " warmup=" + str(warmup))
                fps_t0 = now_ms()
                rc, stdout, stderr = run_smoke([
                    "--runs", str(runs),
                    "--warmup", str(warmup),
                ])
                fps_host_ms = now_ms() - fps_t0
                regs[9] = rc & 0xFFFFFFFF
                log_smoke_summary(stdout + "\n" + stderr)
                populate_tpu_stats(stdout + "\n" + stderr)
                if rc != 0:
                    log_line(
                        "fps exit=" + str(rc) +
                        " host_ms=" + str(fps_host_ms))
                    log_line(stderr)
                    set_status(False)
                else:
                    detections = parse_detections(stdout + "\n" + stderr)
                    count = parse_detection_count(stdout + "\n" + stderr)
                    if count <= 0:
                        count = len(detections)
                    regs[7] = count
                    regs[8] = parse_kv_int(stdout, "measured_ms") & 0xFFFFFFFF
                    regs[10] = parse_kv_int(stdout, "completed") & 0xFFFFFFFF
                    regs[11] = warmup & 0xFFFFFFFF
                    regs[12] = parse_kv_int(stdout, "fps_x100") & 0xFFFFFFFF
                    regs[13] = parse_kv_int(stdout, "invoke_ms_sum") & 0xFFFFFFFF
                    log_line(
                        "fps ok completed_runs=" + str(regs[10]) +
                        " fps_x100=" + str(regs[12]) +
                        " detection_count=" + str(count) +
                        " parsed_detections=" + str(len(detections)) +
                        " host_ms=" + str(fps_host_ms) +
                        " measured_ms=" + str(regs[8]) +
                        " invoke_ms_sum=" + str(regs[13])
                    )
                    set_status(regs[10] > 0 and count > 0 and len(detections) > 0)
            elif cmd == 4:
                det_index = kind
                if det_index < 0 or det_index >= len(detections):
                    set_status(False)
                else:
                    det = detections[det_index]
                    regs[10] = det[0] & 0xFFFFFFFF
                    regs[11] = det[1] & 0xFFFFFFFF
                    regs[12] = det[2] & 0xFFFFFFFF
                    regs[13] = det[3] & 0xFFFFFFFF
                    regs[14] = det[4] & 0xFFFFFFFF
                    regs[15] = det[5] & 0xFFFFFFFF
                    set_status(True)
            else:
                set_status(False)
        except Exception as e:
            try:
                log_line("exception " + str(e))
            except Exception:
                pass
            set_status(False)
