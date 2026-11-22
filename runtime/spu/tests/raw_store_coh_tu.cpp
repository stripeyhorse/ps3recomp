/* Exercises the raw store paths of the generated ppu_recomp.h exactly as lifted code uses them. */
#include "ppu_recomp.h"
extern "C" void fx_stw_raw(uint32_t ea, uint32_t v)   { VM_RAW_ST(uint32_t, ea, v); }          /* stwbrx shape */
extern "C" void fx_stvx(uint32_t ea, const void* src) { vm_raw_st16(ea, src); }                 /* stvx */
extern "C" void fx_dcbz(uint32_t ea)                  { vm_dcbz(ea); }                         /* dcbz */
