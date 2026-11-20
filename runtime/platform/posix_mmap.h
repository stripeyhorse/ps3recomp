#ifndef PS3RECOMP_POSIX_MMAP_H
#define PS3RECOMP_POSIX_MMAP_H

#include <sys/mman.h>

/* Anonymous mappings are demand-paged. Where available, MAP_NORESERVE also
 * avoids an up-front swap reservation for the guest address space. */
#ifdef MAP_NORESERVE
#define PS3_MAP_PRIVATE_ANON (MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE)
#else
#define PS3_MAP_PRIVATE_ANON (MAP_PRIVATE | MAP_ANONYMOUS)
#endif

#endif
