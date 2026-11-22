#!/usr/bin/env python3
"""MFC engine binding (PS3_SPU_MFC_BIND) and collapsed MFC diagnostics
(PS3_SPU_MFC_FAST): lifetime checks, then one guest-visible hash across every
switch combination, then concurrent claim/release.

Set MFC_BIND_SANITIZE=address,undefined or =thread to build with sanitizers."""
import os
import shlex
import subprocess
import tempfile
from pathlib import Path

if os.name == "nt":
    print("SKIP: MFC binding fixture currently requires a POSIX C compiler")
    raise SystemExit(0)
root = Path(__file__).resolve().parents[1]
spu = root / "runtime/spu"
common = ["spu_drain.c", "spu_lockstep.c", "spu_coherency.c", "spu_workload.c",
          "spurs_policy.c", "spurs_job.c", "spu_tsp_weak.c", "spu_vm_pagemap.c",
          "spurs_policy_blob_weak.c", "spu_interp.c"]
env = {k: v for k, v in os.environ.items()
       if not k.startswith(("SPU_", "SPURS_", "YDKJ_", "LBP_", "GH3_", "PS3_SPU_MFC_"))}
env["ASAN_OPTIONS"] = "detect_leaks=0"
san = os.environ.get("MFC_BIND_SANITIZE", "")


def run(binary, cwd, *args, extra=None):
    p = subprocess.run([str(binary), *args], env={**env, **(extra or {})},
                       cwd=cwd, capture_output=True, text=True, timeout=600)
    if p.returncode:
        print(p.stdout, end="")
        print(p.stderr)
        raise SystemExit(f"{' '.join(args) or 'lifetime'} {extra or ''}: exit {p.returncode}")
    if "WARNING: ThreadSanitizer" in p.stderr or "runtime error:" in p.stderr:
        print(p.stderr)
        raise SystemExit("sanitizer report")
    return p.stdout


with tempfile.TemporaryDirectory(prefix="spu-mfc-bind-") as tmp:
    binary = Path(tmp) / "fixture"
    flags = ["-O1", "-g"] + ([f"-fsanitize={san}", "-fno-omit-frame-pointer"] if san else [])
    subprocess.run([os.environ.get("CC", "cc"), "-std=gnu17", "-UNDEBUG", *flags,
                    *shlex.split(os.environ.get("CFLAGS", "")), "-pthread",
                    "-I" + str(root / "include"), "-I" + str(spu),
                    "-I" + str(root / "runtime/platform"),
                    str(spu / "tests/test_spu_mfc_bind.c"),
                    *[str(spu / name) for name in common],
                    str(root / "runtime/platform/win32_compat.c"), "-lm",
                    "-o", str(binary)], check=True)

    print(run(binary, tmp), end="")

    steps = os.environ.get("MFC_BIND_STEPS", "20000")
    hashes = {}
    for bind in ("1", "0"):
        for fast in ("1", "0"):
            for extra in ({}, {"SPU_DMACHK": "1"}):
                mode = {"PS3_SPU_MFC_BIND": bind, "PS3_SPU_MFC_FAST": fast, **extra}
                out = run(binary, tmp, "diff", steps, extra=mode).split()
                hashes[" ".join(f"{k}={v}" for k, v in mode.items())] = out[2]
    for mode, h in hashes.items():
        print(f"  {h}  {mode}")
    assert len(set(hashes.values())) == 1, "guest-visible state differs between modes"
    print(f"PASS: {len(hashes)} switch combinations give one guest-visible hash over {steps} steps")

    for bind in ("1", "0"):
        print(run(binary, tmp, "stress", extra={"PS3_SPU_MFC_BIND": bind}), end="")
    print("PASS: MFC engine binding lifetime, equivalence and concurrency")
