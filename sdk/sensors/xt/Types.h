/*
 * Copyright 2019 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef Types_H
#define Types_H

#include <stddef.h>
#include <limits.h>
#include <float.h>
#include <sys/types.h>

typedef char            Char;
// Pointer-sized integer
typedef size_t          UPInt;
typedef ptrdiff_t       SPInt;
typedef int             SByte  __attribute__((__mode__ (__QI__)));
typedef unsigned int    UByte  __attribute__((__mode__ (__QI__)));
typedef int             SInt16 __attribute__((__mode__ (__HI__)));
typedef unsigned int    UInt16 __attribute__((__mode__ (__HI__)));
typedef int             SInt32 __attribute__((__mode__ (__SI__)));
typedef unsigned int    UInt32 __attribute__((__mode__ (__SI__)));
typedef int             SInt64 __attribute__((__mode__ (__DI__)));
typedef unsigned int    UInt64 __attribute__((__mode__ (__DI__)));

// Byte order constants, LAR_BYTE_ORDER is defined to be one of these.
#define LAR_LITTLE_ENDIAN       1
#define LAR_BIG_ENDIAN          2

#define LAR_FORCE_INLINE  inline __attribute__((always_inline))
#define LAR_BYTE_ORDER    LAR_LITTLE_ENDIAN

// Assembly macros
#define LAR_ASM                  __asm__
#define LAR_ASM_PROC(procname)   LAR_ASM
#define LAR_ASM_END              LAR_ASM

#define LAR_ASSERT(p)    ((void)0)

// Compile-time assert; produces compiler error if condition is false
#define LAR_COMPILER_ASSERT(x)  { int zero = 0; switch(zero) {case 0: case x:;} }
#  define   LAR_UNUSED(a)   do {__typeof__ (&a) __attribute__ ((unused)) __tmp = &a; } while(0)

#define     LAR_UNUSED1(a1) LAR_UNUSED(a1)
#define     LAR_UNUSED2(a1,a2) LAR_UNUSED(a1); LAR_UNUSED(a2)
#define     LAR_UNUSED3(a1,a2,a3) LAR_UNUSED2(a1,a2); LAR_UNUSED(a3)
#define     LAR_UNUSED4(a1,a2,a3,a4) LAR_UNUSED3(a1,a2,a3); LAR_UNUSED(a4)
#define     LAR_UNUSED5(a1,a2,a3,a4,a5) LAR_UNUSED4(a1,a2,a3,a4); LAR_UNUSED(a5)
#define     LAR_UNUSED6(a1,a2,a3,a4,a5,a6) LAR_UNUSED4(a1,a2,a3,a4); LAR_UNUSED2(a5,a6)
#define     LAR_UNUSED7(a1,a2,a3,a4,a5,a6,a7) LAR_UNUSED4(a1,a2,a3,a4); LAR_UNUSED3(a5,a6,a7)
#define     LAR_UNUSED8(a1,a2,a3,a4,a5,a6,a7,a8) LAR_UNUSED4(a1,a2,a3,a4); LAR_UNUSED4(a5,a6,a7,a8)
#define     LAR_UNUSED9(a1,a2,a3,a4,a5,a6,a7,a8,a9) LAR_UNUSED4(a1,a2,a3,a4); LAR_UNUSED5(a5,a6,a7,a8,a9)
#endif 
