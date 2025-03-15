/*
 * types.h
 *
 *  Created on: 01 oct. 2022
 *      Author: Ludo
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include "stdint.h"

/*!******************************************************************
 * \brief Custom variables types.
 *******************************************************************/

typedef char                char_t;

typedef signed long long    int64_t;
typedef unsigned long long  uint64_t;

typedef float               float32_t;
typedef double              float64_t;

#define NULL                ((void*) 0)

#define UNUSED(x)           ((void) x)

#endif /* __TYPES_H__ */
