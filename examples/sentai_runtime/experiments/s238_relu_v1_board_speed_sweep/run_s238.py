#!/usr/bin/env python3
"""s238 - on-board speed sweep of the 8 ReLU (relu_v1) cover detectors.

Re-does the s236 measurement for the ReLU-activation models (MLflow
sentai_revision_relu_v1, exp 8), fed a REAL image (test.bin = the s236-chosen
640x480 RGB u8 frame that detects humans) via sentai.tpu.load_image, on the
physical i.MX RT1176 board.

Honest wall-clock timing per s236: N invokes timed with sentai.rtos.micros()
(microsecond wall clock), ms_per_invoke = wall_us / (1000*N). Prints the URB
per-phase breakdown (instruction upload / input transfer / output read wait)
so the ms are transparent -- no fps-from-a-crash artefacts.

Unlike s236 there is NO sys.reset() between models: the ReLU graphs have
scratch_size_bytes=0 (verified offline), so none spills / wedges the TPU, and
all 8 run back-to-back in one REPL session.

Run:  venv-flash/bin/python3 run_s238.py
"""
import sys, os, time, csv, re

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "s236_model_speed_sweep_640x480"))
import board_serial as B

MODELS = ["msblock", "c2f", "c3", "c2f_pan2", "gelan", "gelan_pan2", "c2f_deep", "c2f_thick"]
N = 50
WARMUP = 5

# on-board bench fn (one exec string; \n-indented). Uses micros() wall clock +
# URB stats. TAGS order: instructions,input,parameters,output,event,unknown;
# per-row field 3 = bytes_done, 12 = wait_us.
# board prints RAW ints only (no on-device float formatting -> avoids MP % type
# errors); host computes ms/fps. Fields after 'RESULT p N':
#   wall_us fails instr_bytes instr_wait_us input_bytes input_wait_us output_bytes output_wait_us
BENCH = (
    "def bench(p,N):\n"
    " rc=sentai.tpu.load(p)\n"
    " if rc!=0:\n"
    "  print('RESULT',p,N,-1,rc,0,0,0,0,0,0);return\n"
    " sentai.tpu.load_image('/test.bin')\n"
    " for _ in range(%d): sentai.tpu.invoke()\n"
    " sentai.tpu.urb_stats(1)\n"
    " f=0;t0=sentai.rtos.micros()\n"
    " for _ in range(N):\n"
    "  if sentai.tpu.invoke()<0:f+=1\n"
    " w=(sentai.rtos.micros()-t0)&0xffffffff\n"
    " hz,rows=sentai.tpu.urb_stats(0)\n"
    " print('RESULT',p,N,w,f,rows[0][3],rows[0][12],rows[1][3],rows[1][12],rows[3][3],rows[3][12])\n"
    % WARMUP
)


def main():
    ser = B.open_repl()
    B.cmd(ser, "import sentai", 4)
    B.cmd(ser, "sentai.verbose(1)", 2)   # CRITICAL: verbose(0) silences all REPL output
    B.cmd(ser, "exec(%r)" % BENCH, 3)

    rows = []
    log = open(os.path.join(HERE, "sweep.log"), "w")
    for m in MODELS:
        out = B.cmd(ser, 'bench("/%s.tflite", %d)' % (m, N), 30)
        log.write(out)
        line = next((l for l in out.split("\n") if l.strip().startswith("RESULT")), None)
        r = parse(line, m) if line else None
        if r:
            print("  %-11s ms=%.3f fps=%.2f fails=%s  (instr %s ms / input %s ms / output %s ms)"
                  % (m, r["ms"], r["fps"], r["fails"], r["instr_ms"], r["input_ms"], r["output_ms"]))
            rows.append(r)
        else:
            print("  %-11s (no RESULT)" % m)
    log.close()

    out_csv = os.path.join(HERE, "results.csv")
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["model", "N", "wall_us", "ms_per_invoke", "fps", "fails",
                    "instr_bytes", "instr_ms", "input_bytes", "input_ms",
                    "output_bytes", "output_ms"])
        for r in rows:
            w.writerow([r["model"], N, r["wall_us"], "%.3f" % r["ms"], "%.2f" % r["fps"],
                        r["fails"], r["instr_b"], "%.3f" % r["instr_ms"], r["input_b"],
                        "%.3f" % r["input_ms"], r["output_b"], "%.3f" % r["output_ms"]])
    print("\nwrote", out_csv)
    ser.close()


def parse(line, m):
    # RESULT /model.tflite N wall_us fails i_b i_wus n_b n_wus o_b o_wus
    toks = line.strip().split()
    idx = toks.index("RESULT")
    _, path, Nn, wall, fails, ib, iwus, nb, nwus, ob, owus = toks[idx:idx + 11]
    wall = int(wall); Nn = int(Nn)
    perinv = lambda wus: int(wus) / 1000.0 / Nn
    return {"model": m, "wall_us": wall, "fails": fails,
            "ms": wall / 1000.0 / Nn if wall > 0 else 0.0,
            "fps": 1e6 * Nn / wall if wall > 0 else 0.0,
            "instr_b": ib, "instr_ms": perinv(iwus),
            "input_b": nb, "input_ms": perinv(nwus),
            "output_b": ob, "output_ms": perinv(owus)}


if __name__ == "__main__":
    main()
