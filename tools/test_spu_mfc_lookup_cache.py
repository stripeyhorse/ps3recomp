#!/usr/bin/env python3
"""Exercise real registry locks, cached lifetime and existing multi-owner DMA."""
import os,subprocess,tempfile
from pathlib import Path
if os.name=='nt':
 print('SKIP: guarded MFC cache fixtures require POSIX mmap/pthreads');raise SystemExit(0)
root=Path(__file__).resolve().parents[1];spu=root/'runtime/spu'
common=['spu_drain.c','spu_lockstep.c','spu_coherency.c','spu_workload.c','spurs_policy.c','spurs_job.c','spu_tsp_weak.c','spu_vm_pagemap.c','spurs_policy_blob_weak.c','spu_interp.c']
with tempfile.TemporaryDirectory(prefix='spu-mfc-cache-') as tmp:
 exe=Path(tmp)/'cache'
 flags=['-std=gnu17','-O2','-UNDEBUG']
 if os.environ.get('GUARD_ALLOCATOR')=='1':flags+=['-DMFC_CACHE_GUARDED_ALLOCATOR']
 if os.environ.get('SANITIZERS'):flags+=['-fsanitize='+os.environ['SANITIZERS'],'-fno-sanitize-recover=all','-g']
 cmd=[os.environ.get('CC','cc'),*flags,'-I'+str(root/'include'),'-I'+str(spu),'-I'+str(root/'runtime/platform'),str(spu/'tests/test_spu_mfc_lookup_cache.c'),*[str(spu/c) for c in common],str(root/'runtime/platform/win32_compat.c'),'-pthread','-lm','-o',str(exe)]
 subprocess.run(cmd,check=True)
 for mode, image, scoped in ((None,None,False),('0','2x',False),
                              ('0','2147483648',False),('0','2',True),('1','2',False)):
  env=dict(os.environ)
  env.pop('SPU_MFC_LOOKUP_CACHE',None)
  env.pop('SPU_MFC_ASYNC_CACHE_IMAGE',None)
  if mode is not None:env['SPU_MFC_LOOKUP_CACHE']=mode
  if image is not None:env['SPU_MFC_ASYNC_CACHE_IMAGE']=image
  argv=[str(exe),'scoped'] if scoped else [str(exe)]
  try:p=subprocess.run(argv,env=env,capture_output=True,text=True,timeout=60)
  except subprocess.TimeoutExpired as error:
   print('timeout mode',mode,'stdout',error.stdout,'stderr',error.stderr,flush=True)
   raise
  print('global',mode,'async image',image,flush=True);print(p.stdout,flush=True)
  if p.returncode:print(p.stderr);raise SystemExit(p.returncode)
