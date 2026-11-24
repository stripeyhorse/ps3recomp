#!/usr/bin/env python3
"""ps3_host_quit (cellSysutil.c) sends REQUEST_EXITGAME, then ends a title
that does not exit by itself with status 0, after any save in flight; asked
directly, and by SIGTERM (a later SIGINT is ignored)."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile


if sys.platform == "win32":
    print("SKIP: host quit fixture requires a POSIX C toolchain")
    raise SystemExit(0)

root = Path(__file__).resolve().parents[1]
env = {key: value for key, value in os.environ.items()
       if not key.startswith("MSGDIALOG_")}
env["PS3_QUIT_GRACE_MS"] = "100"
gc_flag = "-Wl,-dead_strip" if sys.platform == "darwin" else "-Wl,--gc-sections"
with tempfile.TemporaryDirectory(prefix="host-quit-") as temporary:
    binary = Path(temporary) / "test_host_quit"
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=gnu17", "-O1", "-UNDEBUG",
        "-ffunction-sections", "-fdata-sections", "-I" + str(root / "include"),
        str(root / "libs/system/tests/test_host_quit.c"),
        str(root / "libs/system/cellSysutil.c"),
        "-pthread", gc_flag, "-o", str(binary),
    ], check=True, timeout=60, env=env, cwd=temporary)
    for mode in ([], ["signal"]):
        result = subprocess.run([str(binary)] + mode, timeout=10, env=env, cwd=temporary,
                                capture_output=True, text=True)
        print(result.stdout.strip())
        requested = result.stdout.count("[quit] requested")
        if result.returncode != 0 or "[PASS]" not in result.stdout or requested != 1 or (
                mode and "[quit] requested (signal SIGTERM)" not in result.stdout):
            raise SystemExit("test_host_quit %s failed (exit %d, %d quit requests)"
                             % (mode[0] if mode else "direct", result.returncode, requested))
