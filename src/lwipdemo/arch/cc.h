#ifndef LWIPDEMO_ARCH_CC_H
#define LWIPDEMO_ARCH_CC_H

#include <stdint.h>
#include <stddef.h>

#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

typedef uint32_t sys_prot_t;

#define LWIP_NO_INTTYPES_H 1
#define LWIP_NO_LIMITS_H   1
#define LWIP_NO_UNISTD_H   1
#define LWIP_NO_CTYPE_H    1

#define X8_F  "02x"
#define U16_F "d"
#define S16_F "d"
#define X16_F "x"
#define U32_F "d"
#define S32_F "d"
#define X32_F "x"
#define SZT_F "d"

#define LWIP_PLATFORM_DIAG(x) do { } while (0)
#define LWIP_PLATFORM_ASSERT(x) do { } while (0)

#define SYS_ARCH_DECL_PROTECT(lev) sys_prot_t lev
#define SYS_ARCH_PROTECT(lev) do { (lev) = 0; } while (0)
#define SYS_ARCH_UNPROTECT(lev) do { (void)(lev); } while (0)

#endif
