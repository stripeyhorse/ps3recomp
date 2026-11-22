#!/usr/bin/env python3
"""SPU_FLOAT_RZ: FPCR is round toward zero inside spu_run_with_halt and the caller's afterwards."""
import os
import subprocess
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
with tempfile.TemporaryDirectory(prefix='spu-float-rz-mode-') as tmp:
    def build(name, *defs):
        binary = Path(tmp) / name
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu17', '-O1', '-UNDEBUG', *defs,
                        '-I'+str(root/'include'), '-I'+str(spu),
                        '-I'+str(root/'runtime/platform'),
                        str(spu/'tests/test_spu_float_rz_mode.c'),
                        *[str(spu / name) for name in sources],
                        str(root/'runtime/platform/win32_compat.c'), '-pthread', '-lm',
                        '-o', str(binary)], check=True)
        return binary
    # The plain build follows SPU_FLOAT_RZ; with an SPU_FLOAT_RZ_ONLY TU linked in, every run
    # is RZ and SPU_FLOAT_RZ=0 is refused with a message.
    for binary, modes in ((build('test'), ('0', '1')), (build('test-rz-only', '-DSPU_FLOAT_RZ_ONLY'), ('0', '1', None))):
        for rz in modes:
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith(('SPU_', 'SPURS_', 'YDKJ_', 'LBP_', 'GH3_'))}
            if rz is not None:
                env['SPU_FLOAT_RZ'] = rz
            result = subprocess.run([str(binary)], capture_output=True, text=True,
                                    timeout=10, cwd=tmp, env=env)
            print(result.stdout.strip())
            if result.returncode:
                print(result.stderr[-2500:])
                raise SystemExit(f'FAILED {binary.name} with SPU_FLOAT_RZ={rz}')
            if binary.name == 'test-rz-only' and rz == '0' and 'SKIP' not in result.stdout \
                    and 'SPU_FLOAT_RZ=0 ignored' not in result.stderr:
                raise SystemExit('RZ-only build did not report the refused SPU_FLOAT_RZ=0')
