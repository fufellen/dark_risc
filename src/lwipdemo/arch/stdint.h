#ifndef LWIPDEMO_ARCH_STDINT_H
#define LWIPDEMO_ARCH_STDINT_H

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;
typedef signed int intptr_t;
typedef unsigned int uintptr_t;
typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;

#define UINT8_MAX  0xffu
#define UINT16_MAX 0xffffu
#define UINT32_MAX 0xffffffffu
#define UINT64_MAX 0xffffffffffffffffull

#endif
