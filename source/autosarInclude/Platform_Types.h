/* -------------------------------- Arctic Core ------------------------------
 * Arctic Core - the open source AUTOSAR platform http://arccore.com
 *
 * Copyright (C) 2009  ArcCore AB <contact@arccore.com>
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation; See <http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt>.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 * -------------------------------- Arctic Core ------------------------------*/

/** @addtogroup General General
 *  @{ */

/** @file Platform_Types.h
 * General platform type definitions.
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

#define CPU_TYPE            CPU_TYPE_32 
#define CPU_BIT_ORDER       MSB_FIRST 
#define CPU_BYTE_ORDER      HIGH_BYTE_FIRST

#ifndef FALSE
#define FALSE		(boolean)false
#endif
#ifndef TRUE
#define TRUE		(boolean)true
#endif
#ifndef boolean
typedef _Bool      			boolean;
#endif
#ifndef sint8
typedef int8_t         		sint8;
#endif
#ifndef uint8
typedef uint8_t       		uint8;
#endif
#ifndef char_t
typedef char				char_t;
#endif
#ifndef sint16
typedef int16_t        		sint16;
#endif
#ifndef uint16
typedef uint16_t      		uint16;
#endif
#ifndef sint32
typedef int32_t         	sint32;
#endif
#ifndef uint32
typedef uint32_t       		uint32;
#endif
#ifndef sint64
typedef int64_t  			sint64;
#endif
#ifndef uint64
typedef uint64_t  			uint64;
#endif
#ifndef uint8_least
typedef uint_least8_t       uint8_least;
#endif
#ifndef uint16_least
typedef uint_least16_t      uint16_least;
#endif
#ifndef uint32_least
typedef uint_least32_t      uint32_least;
#endif
#ifndef sint8_least
typedef int_least8_t        sint8_least;
#endif
#ifndef sint16_least
typedef int_least16_t       sint16_least;
#endif
#ifndef sint32_least
typedef int_least32_t       sint32_least;
#endif
#ifndef float32
typedef float               float32; 
#endif
#ifndef float64
typedef double              float64;  
#endif

#ifndef vint8_t
typedef volatile int8_t vint8_t;
#endif
#ifndef vuint8_t
typedef volatile uint8_t vuint8_t;
#endif
#ifndef vint16_t
typedef volatile int16_t vint16_t;
#endif
#ifndef vuint16_t
typedef volatile uint16_t vuint16_t;
#endif
#ifndef vint32_t
typedef volatile int32_t vint32_t;
#endif
#ifndef vuint32_t
typedef volatile uint32_t vuint32_t;
#endif
#ifndef vint64_t
typedef volatile int64_t vint64_t;
#endif
#ifndef vuint64_t
typedef volatile uint64_t vuint64_t;
#endif

#endif
/** @} */
