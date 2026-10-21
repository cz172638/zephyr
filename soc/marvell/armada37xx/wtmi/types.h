/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Replacement for A3700-utils wtmi/common/types.h: the Marvell code's
 * integer types on top of the C library ones (bool from stdbool.h).
 */

#ifndef A3700_WTMI_TYPES_H
#define A3700_WTMI_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef char int_8;
typedef unsigned char uint_8;
typedef short int_16;
typedef unsigned short uint_16;
typedef int int_32;
typedef unsigned int uint_32;
typedef long long int_64;
typedef unsigned long long uint_64;
typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef long long s64;
typedef int s32;
typedef short s16;
typedef char s8;
typedef char byte;

/* Errors */
#define NO_ERROR              0x0000
#define ERR_INVALID_ARGUMENT  0x0001
#define ERR_INVALID_COMMAND   0x0002
#define ERR_BAD_CONFIGURATION 0x0003
#define ERR_TIMEOUT           0x0004

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#endif /* A3700_WTMI_TYPES_H */
