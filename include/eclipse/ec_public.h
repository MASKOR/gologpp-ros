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
 * $Id: ec_public.h,v 1.8 2017/02/09 23:36:39 jschimpf Exp $
 *
 * Macro definitions needed for the ECLiPSe embedding interface.
 *
 */


#include "ec_general.h"


/*
 * If the word size is big enough, doubles are unboxed
 */

#if SIZEOF_WORD >= SIZEOF_DOUBLE
#define UNBOXED_DOUBLES
#else
#undef UNBOXED_DOUBLES
#endif


/*
 * Sometimes it's useful to know how large an integer we can fit in a
 * double.
 * The following probably should be autoconf'ed.
 * An IEEE double precision floating point number has a 52 bit significand,
 * which coupled with an implicit 1, means we have 53 bits to play with.
 * This means that 2^53 - 1 is the largest integer representable with all
 * its bits stored explicitly, 2^53 works (the bit that drops off the end is
 * a 0, which gets assumed), and 2^53 + 1 is the first one to really fail.
 * Of course, these integers cannot be represented on a 32-bit machine, so
 * we provide the same limit as a double, but in this case we don't know
 * whether 2^53 really was 2^53 before conversion, or whether it's 2^53 + 1
 * rounded down.
 */
#define DOUBLE_SIGNIFICANT_BITS	53
#if (SIZEOF_WORD >= SIZEOF_DOUBLE)
#define DOUBLE_INT_LIMIT	(WSUF(1) << DOUBLE_SIGNIFICANT_BITS)
#endif
#define DOUBLE_INT_LIMIT_AS_DOUBLE	9007199254740992.0

/*
 * Global array sizes
 */

#define MAXARITY		255	/* Max arity of a regular predicate */
#define MAXSIMPLEARITY		8	/* Max arity of a simple predicate */
#define NARGREGS		(1/*A0*/ + MAXARITY + MAXSIMPLEARITY + 3/*metacall*/)
#define NTYPES			13	/* Number of types */
#define ARITH_OPERATIONS	53	/* Number of arithmetic operations */


/*
 * The most common return values from C externals
 * and from resumed Eclipse executions.  Errors are negative.
 */

#define PSUCCEED		0	/* success			*/
#define PFAIL			1	/* failure			*/
#define PTHROW			2	/* throw/1, ball in A1		*/
#define PEXITED			3	/* engine exited explicitly	*/
#define PYIELD			4	/* engine suspended		*/
#define PRUNNING		5	/* engine running asynchronously*/
#define PWAITIO			6	/* waiting for queue input	*/
#define PFLUSHIO		7	/* request queue output		*/

#define EC_EXTERNAL_ERROR	-3	/* error in a user-defined external */
#define INSTANTIATION_FAULT	-4	/* variable instead of constant */
#define TYPE_ERROR		-5	/* wrong type */
#define RANGE_ERROR		-6	/* out of range */


/*
 * Initialisation
 *
 * init_flags indicates what parts of the system need to be
 * initialised (bit-significant flags):
 *
 *	INIT_SHARED	shared/saveable heap
 *	REINIT_SHARED	heap was restored, some info must be updated
 *	INIT_PRIVATE	C variables, private heap
 *	INIT_PROCESS	do initialisations that are needed once
 *
 *	INIT_WITH_PROFILER	use instrumented emulator
 *
 * Options for engine initialization:
 *
 *	INIT_ASYNC	init engine with its own thread
 *	INIT_NO_MAIN	don't call main/1 in new engine
 *	INIT_CLONE	init engine from the parent engine
 *
 * Initialisation is done in different situations:
 *
 * raw boot		INIT_SHARED|INIT_PRIVATE|INIT_PROCESS
 * after -r		REINIT_SHARED|INIT_PROCESS|INIT_PRIVATE
 * after -c		INIT_PROCESS|INIT_PRIVATE
 */

#define	INIT_SHARED	1
#define	REINIT_SHARED	2
#define	INIT_PRIVATE	4
#define	INIT_PROCESS	8
#define	INIT_NO_MAIN	16
#define	INIT_ASYNC	32
#define	INIT_CLONE	64
#define	INIT_WITH_PROFILER	128	/* with INIT_SHARED */


