#!/usr/bin/env python3
"""Run the B7 read-only model-substrate smoke check in sentai_sim."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIM = ROOT / "build-sim/sim/sentai_sim"
DEFAULT_FS_ROOT = ROOT / "build-sim/sentai_fs_root"
SMOKE_SRC = ROOT / "examples/sentai_runtime/diag/smoke_namespace_model.py"
SMOKE_DST = "smoke_namespace_model.py"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run read-only sentai model namespace smoke in SIM"
    )
    parser.add_argument("--sim", type=Path, default=DEFAULT_SIM)
    parser.add_argument("--fs-root", type=Path, default=DEFAULT_FS_ROOT)
    args = parser.parse_args()

    if not args.sim.exists():
        raise SystemExit(f"missing simulator binary: {args.sim}")
    if not args.fs_root.exists():
        raise SystemExit(f"missing SIM fs root: {args.fs_root}")

    shutil.copyfile(SMOKE_SRC, args.fs_root / SMOKE_DST)
    code = "import sentai\nsentai.run('/" + SMOKE_DST + "')\n"
    proc = subprocess.run(
        [str(args.sim)],
        input=code,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=str(ROOT),
        check=False,
    )
    print(proc.stdout, end="")
    if proc.returncode != 0:
        return proc.returncode
    if "OK namespace_model_smoke" not in proc.stdout:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
