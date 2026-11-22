#!/usr/bin/env python3
"""PS3_SPU_IB_FAST: the indirect-branch target cache leaves guest state unchanged.

Every scenario of runtime/spu/tests/test_spu_ib_fast.c runs with the cache off,
on and in shadow mode; all three must print the same digest, shadow mode must
count no mismatch, and the scenarios that branch to the same targets repeatedly
must hit. `--bench` also prints ns per indirect branch for the old and new path.
Set SANITIZE=1 to build with ASan and UBSan."""
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

if os.name == 'nt':
    print('SKIP: fixture currently requires the POSIX compiler/linker')
    raise SystemExit(0)
root = Path(__file__).resolve().parents[1]
spu = root / 'runtime/spu'
sources = ['spu_channels.c', 'spu_drain.c', 'spu_lockstep.c', 'spu_coherency.c',
           'spu_workload.c', 'spurs_policy.c', 'spurs_job.c', 'spu_tsp_weak.c',
           'spu_vm_pagemap.c', 'spurs_policy_blob_weak.c', 'spu_interp.c']
SCENARIOS = {  # name: must hit in shadow mode
    'residency': True, 'reloc': True, 'task': True, 'switch': True, 'special': True,
    'ls0': True, 'stackreset': True, 'irq': True, 'calls': True,
}
bench = '--bench' in sys.argv
sanitize = os.environ.get('SANITIZE') == '1'
with tempfile.TemporaryDirectory(prefix='spu-ib-fast-') as tmp:
    binary = Path(tmp) / 'test'
    flags = ['-O2', '-g'] + (['-fsanitize=address,undefined', '-fno-sanitize-recover=undefined',
                              '-fno-omit-frame-pointer'] if sanitize else [])
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu17', *flags, '-UNDEBUG',
                    '-I' + str(root / 'include'), '-I' + str(spu),
                    '-I' + str(root / 'runtime/platform'),
                    str(spu / 'tests/test_spu_ib_fast.c'),
                    *[str(spu / name) for name in sources],
                    str(root / 'runtime/platform/win32_compat.c'), '-pthread', '-lm',
                    '-o', str(binary)], check=True)
    base_env = {k: v for k, v in os.environ.items()
                if not k.startswith(('SPU_', 'SPURS_', 'PS3_SPU_', 'YDKJ_', 'LBP_', 'GH3_'))}
    if sanitize:
        base_env['ASAN_OPTIONS'] = 'detect_leaks=0'

    def run(args, mode):
        env = dict(base_env)
        env['PS3_SPU_IB_FAST'] = mode
        r = subprocess.run([str(binary), *args], capture_output=True, text=True,
                           timeout=600, cwd=tmp, env=env)
        return r

    failed = []
    for name, must_hit in SCENARIOS.items():
        digests = {}
        for mode in ['0', '1', 'shadow']:
            r = run([name], mode)
            line = next((l for l in r.stdout.splitlines() if l.startswith(name + ':')), '')
            print(f'{mode:>6} {line}')
            if name == 'irq':
                print('       ' + next((l for l in r.stdout.splitlines() if l.startswith('irq records')), ''))
            if r.returncode or 'PASS' not in r.stdout:
                print(r.stdout[-1500:] + r.stderr[-2500:])
                failed.append(f'{name}/{mode}')
                continue
            m = re.search(r'digest=(\w+).*hits=(\d+) misses=(\d+) inserts=(\d+) mismatches=(\d+) escapes=(\d+)', line)
            digests[mode] = m.group(1)
            if mode == 'shadow':
                hits, mism, esc = int(m.group(2)), int(m.group(5)), int(m.group(6))
                if mism or esc or (must_hit and not hits):
                    failed.append(f'{name}/shadow hits={hits} mismatches={mism} escapes={esc}')
        if len(set(digests.values())) != 1 or len(digests) != 3:
            failed.append(f'{name}: digests differ {digests}')
    if bench:
        for mode in ['0', '1', '0', '1']:
            r = run(['bench'], mode)
            print(f'{mode:>6} ' + (r.stdout.strip() or r.stderr[-500:]))
    if failed:
        raise SystemExit('FAILED: ' + '; '.join(failed))
    print('PASS: indirect-branch target cache matches the full resolver in every scenario')
