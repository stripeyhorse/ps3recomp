#!/usr/bin/env python3
"""Vulkan frame-dump queue/writer (libs/video/rsx_vk_dump.h): non-blocking enqueue, drop on overflow, atomic PPM."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
root=Path(__file__).resolve().parents[1]
if os.name=='nt':
    print('SKIP: POSIX threads fixture')
    raise SystemExit(0)
with tempfile.TemporaryDirectory(prefix='vk-dump-') as tmp:
    binary=Path(tmp)/'test'
    subprocess.run([os.environ.get('CC','cc'),'-std=gnu17','-D_GNU_SOURCE','-O1','-UNDEBUG','-Wall','-Wextra',
        str(root/'tests/test_vk_dump.c'),'-pthread','-o',str(binary)],check=True)
    for _ in range(int(os.environ.get('RUNS','10'))):
        subprocess.run([str(binary)],timeout=60,check=True)
