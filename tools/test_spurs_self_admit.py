#!/usr/bin/env python3
"""Worker self-admission and targeted wakes, with PS3_SPURS_SELF_ADMIT both ways.

SANITIZE=thread|address builds the fixture with that sanitizer (address adds
UndefinedBehaviorSanitizer)."""
import os, platform, subprocess, tempfile
from pathlib import Path
if os.name == 'nt':
    print('SKIP: POSIX fixture'); raise SystemExit(0)
r = Path(__file__).resolve().parents[1]
san = os.environ.get('SANITIZE', '')
flags = {'': ['-O1'], 'thread': ['-O1', '-g', '-fsanitize=thread'],
         'address': ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=undefined']}[san]
with tempfile.TemporaryDirectory(prefix='spurs-self-admit-') as tmp:
    out = Path(tmp)/'test'
    units = ['spu_channels', 'spu_drain', 'spu_lockstep', 'spu_coherency', 'spu_workload',
             'spurs_policy', 'spurs_job', 'spu_tsp_weak', 'spu_vm_pagemap',
             'spurs_policy_blob_weak', 'spu_interp']
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu17', '-D_GNU_SOURCE', *flags, '-UNDEBUG',
        '-ffunction-sections', '-fdata-sections',
        '-Wl,-dead_strip' if platform.system() == 'Darwin' else '-Wl,--gc-sections',
        '-I'+str(r/'include'), '-I'+str(r/'runtime/spu'), '-I'+str(r/'runtime/platform'),
        str(r/'tests/test_spurs_self_admit.c'),
        *[str(r/'runtime/spu'/f'{u}.c') for u in units],
        str(r/'runtime/platform/win32_compat.c'), '-pthread', '-lm', '-o', str(out)], check=True)
    args = [] if not san else ['--quick']
    env0 = dict(os.environ)
    if san == 'thread':
        # TSAN_OPTIONS cannot quote a path with spaces: use a copy in tmp.
        supp = Path(tmp)/'guest_memory.supp'
        supp.write_text((r/'tests/tsan_spurs_guest_memory.supp').read_text())
        env0['TSAN_OPTIONS'] = f'suppressions={supp}'
    for mode in ('1', '0'):
        env = dict(env0, PS3_SPURS_SELF_ADMIT=mode)
        res = subprocess.run([str(out), *args], env=env, timeout=600 if san else 120,
                             capture_output=True, text=True)
        lines = [l for l in res.stdout.splitlines() if not l.startswith('[')]
        print('\n'.join(lines))
        if res.returncode:
            print(res.stderr[-4000:]); raise SystemExit(res.returncode)
        if res.stderr.strip() and san:
            print(res.stderr[-4000:])
