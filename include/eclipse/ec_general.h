/* BEGIN LICENSE BLOCK
 * Version: CMPL 1.1
 *
 * The contents of this file are subject to the Cisco-style Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file except
 * in compliance with the License.  You may obtain a copy of the License
 * at www.eclipse-clp.org/license.
 * 
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied.  See
 * the License for the specific language governing rights and limitations
 * under the License. 
 * 
 * The Original Code is  The ECLiPSe Constraint Logic Programming System. 
 * The Initial Developer of the Original Code is  Cisco Systems, Inc. 
 * Portions created by the Initial Developer are
 * Copyright (C) 1989-2006 Cisco Systems, Inc.  All Rights Reserved.
 * 
 * Contributor(s): 
 * 
 * END LICENSE BLOCK */

/*
 * ECLiPSe INCLUDE FILE
 *
 * $Id: ec_general.h,v 1.5 2016/10/28 22:44:33 jschimpf Exp $
 *
 * General types and macros used in ECLiPSe-related C source.
 * Do not include ECLiPSe-specific definitions here.
 *
 */

#ifndef EC_GENERAL_H
#define EC_GENERAL_H


#ifdef _WIN32

/* Require 0x600=Win2008/Vista (needed for condition variables) */
#if !defined(_WIN32_WINNT) || _WIN32_WINNT < 0x600
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x600
#endif
#include <windows.h>

typedef CRITICAL_SECTION ec_mutex_t;
typedef CONDITION_VARIABLE ec_cond_t;

/* don't use pthreads on Windows, even if suported */
#undef HAVE_PTHREAD_H

#elif defined(HAVE_PTHREAD_H)

#include <pthread.h>
typedef pthread_mutex_t ec_mutex_t;
typedef pthread_cond_t ec_cond_t;

#else

#error "This ECLiPSe version needs thread support"

#endif

#ifdef _WIN32
#define Winapi WINAPI
#else
#define Winapi
#endif


#ifdef __GNUC__

#define ec_atomic_add(p,c)		__sync_add_and_fetch(p,c)
#define ec_atomic_and(p,c)		__sync_and_and_fetch(p,c)
#define ec_atomic_or(p,c)		__sync_or_and_fetch(p,c)
#define ec_atomic_load(p)		(*(p))
#define ec_atomic_store(p,v)		(*(p)=v)
#define ec_compare_and_swap(p,c,v)	__sync_bool_compare_and_swap(p,c,v)
#define ec_memory_barrier

#else

#warning "WARNING: not using atomic load/store primitives (requires gcc)!"
#define ec_atomic_add(p,c)		(*(p)+=(c))
#define ec_atomic_and(p,c)		(*(p)&=(c))
#define ec_atomic_or(p,c)		(*(p)|=(c))
#define ec_atomic_load(p)		(*(p))
#define ec_atomic_store(p,v)		(*(p)=v)
#define ec_compare_and_swap(p,c,v)	(*(p)=v,1)	/* assume this intention */
#define ec_memory_barrier

#endif


/* To allow to compile external include files under C++ */

#if defined(__cplusplus)
#  define Dots	...
#  define Extern extern "C"
#  ifdef const
#    undef const
#  endif
#else
#  define Dots
#  define Extern extern
#endif


#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>


/******************************************************************/
/*		Machine-dependent definitions			 */
/******************************************************************/

/* To define the fixed-size integer types int8_t, int64_t, etc */
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif

#ifndef HAVE_INT128_T
#ifdef HAVE___INT128
#define HAVE_INT128_T
typedef __int128	int128_t;		/* exactly 128 bit */
#endif
#endif

/* define word/uword: pointer-sized integers */
#define ECLIPSE_TYPEDEF_WORD			/* tested in memman.h */
#if (SIZEOF_CHAR_P == SIZEOF_INT)
typedef int		word;			/* pointer-sized */
typedef unsigned int	uword;
#elif (SIZEOF_CHAR_P == SIZEOF_LONG)
typedef long		word;			/* pointer-sized */
typedef unsigned long	uword;
#elif (defined(HAVE_LONG_LONG) || defined(__GNUC__)) && (SIZEOF_CHAR_P == __SIZEOF_LONG_LONG__)
typedef long long 		word;		/* pointer-sized */
typedef unsigned long long 	uword;
#elif defined(HAVE___INT64) && SIZEOF_CHAR_P == 8
typedef __int64          word;
typedef unsigned __int64 uword;
#else
#error "No code for dealing with word size > long long/__int64!"
#endif


/* if possible, define doubleword: a double-word sized integer */
#if (SIZEOF_CHAR_P == 8)
#ifdef HAVE_INT128_T
#define HAVE_DOUBLEWORD
typedef int128_t doubleword;
#endif
#elif (SIZEOF_CHAR_P == 4)
#if defined(HAVE_LONG_LONG) && (__SIZEOF_LONG_LONG__ == 4)
#define HAVE_DOUBLEWORD
typedef long long doubleword;
#elif defined(HAVE___INT64)
#define HAVE_DOUBLEWORD
typedef __int64 doubleword;
#endif
#else
#error "No code for dealing with word size"
#endif


/* A "word" is a pointer-sized integer.
 * The following size/min/max definitions are all about "words",
 * even when they say "int".
 */
#define SIZEOF_WORD	SIZEOF_CHAR_P

/* suffix needed for word-sized constants */
#if (SIZEOF_WORD == SIZEOF_INT)
#define WSUF(X)  X
#define UWSUF(X) X
#define W_MOD ""
#elif (SIZEOF_WORD == SIZEOF_LONG)
#define WSUF(X) (X##L)
#define UWSUF(X) (X##UL)
#define W_MOD "l"
#elif (defined(HAVE_LONG_LONG) || defined(__GNUC__)) && \
     (SIZEOF_WORD == __SIZEOF_LONG_LONG__)
#define WSUF(X) (X##LL)
#define UWSUF(X) (X##ULL)
# ifdef _WIN32
/* MSVC did not support long long until V8 (2005). Cross-compiling with
   MingGW can use long long, but as calling printf/scanf will use MS
   rather than GNU libc, we need to use MS's 64 bit integer modifier
*/ 
# define W_MOD "I64"
# else
# define W_MOD "ll"
# endif
#elif (defined(HAVE___INT64) && SIZEOF_WORD == 8)
#define WSUF(X) (X##I64)
#define UWSUF(X) (X##UI64)
#define W_MOD "I64"
#else
PROBLEM: word size do not correspond to size of common integer types
#endif

/* Important:  SIGN_BIT is unsigned! */
#if (SIZEOF_WORD ==  8)
#define SIGN_BIT	((uword) UWSUF(0x8000000000000000))
#define MAX_NUMBER	"9223372036854775807"
#elif (SIZEOF_WORD == 4)
#define SIGN_BIT	((uword) UWSUF(0x80000000))
#define MAX_NUMBER	"2147483647"
#else
PROBLEM: Cannot deal with word size SIZEOF_WORD.
#endif


#define MAX_U_WORD	((uword) -1)
#define MAX_S_WORD	((word) ~SIGN_BIT)
#define MIN_S_WORD      ((word) SIGN_BIT)

/*
 * Min/max words as doubles.
 * These definitions are guaranteed to work fine regardless of word size as
 * as long as MIN_S_WORD is a power of 2 and MAX_S_WORD is one less than a
 * power of two --- assuming round-to-nearest during type conversion.
 */

#define MIN_S_WORD_DBL		((double)MIN_S_WORD)
#define MAX_S_WORD_1_DBL	((double)MAX_S_WORD+1.0)


#define SIZEOF_DOUBLE	8

typedef union {
	double	as_dbl;
#if (SIZEOF_WORD == 8)
	uword as_int;
#endif
	struct ieee_parts {
# ifdef WORDS_BIGENDIAN 
		uint32_t mant1;
		uint32_t mant0;
# else
		uint32_t mant0;
		uint32_t mant1;
# endif
	} as_struct;
} ieee_double;


#endif
