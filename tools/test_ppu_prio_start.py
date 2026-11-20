#!/usr/bin/env python3
"""PS3_PRIO_START and PS3_CREATOR_FIRST through the real sys_ppu_thread.c, in all four combinations."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
root=Path(__file__).resolve().parents[1]
if os.name=='nt':
    print('SKIP: fixture currently uses POSIX linking')
    raise SystemExit(0)
with tempfile.TemporaryDirectory(prefix='ppu-prio-start-') as tmp:
    binary=Path(tmp)/'test'
    linker=['-Wl,-dead_strip'] if sys.platform=='darwin' else ['-Wl,--gc-sections','-pthread','-lm']
    subprocess.run([os.environ.get('CC','cc'),'-std=gnu17','-D_GNU_SOURCE','-O1','-UNDEBUG',
        '-ffunction-sections','-fdata-sections',
        '-I'+str(root/'include'),'-I'+str(root/'runtime'),'-I'+str(root/'runtime/syscalls'),
        str(root/'tests/test_ppu_prio_start.c'),*linker,'-o',str(binary)],check=True)
    quiet=subprocess.PIPE if os.environ.get('CI') else None
    for on, cf in ((False, False), (True, False), (False, True), (True, True)):
        env={k:v for k,v in os.environ.items() if k not in ('PS3_PRIO_START','PS3_CREATOR_FIRST')}
        if on: env['PS3_PRIO_START']='1'
        if cf: env['PS3_CREATOR_FIRST']='1'
        result=subprocess.run([str(binary)],timeout=60,env=env,stderr=quiet,text=True)
        if result.returncode and result.stderr:
            sys.stderr.write(result.stderr)
        result.check_returncode()
