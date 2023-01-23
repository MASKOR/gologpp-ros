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
 * ECLiPSe system
 *
 * IDENTIFICATION:	os_support.h
 *
 * $Id: os_support.h,v 1.12 2017/09/01 03:05:10 jschimpf Exp $
 *
 * AUTHOR:		Joachim Schimpf, IC-Parc
 *
 * DESCRIPTION:		Operating-system services abstraction layer
 *		
 */

#ifndef OS_SUPPORT_H
#define OS_SUPPORT_H

#include "ec_general.h"

#ifndef DLLEXP
#ifdef _WIN32
/* in case this file is included on its own, otherwise defined in sepia.h */
#define DLLEXP __declspec(dllimport)
#else
#define DLLEXP
#endif
#else
/* use includer's dllexport definition */
#endif


#include <errno.h>
#include <sys/types.h>

#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#ifdef HAVE_STRING_H
#include <string.h>
#endif

#ifdef _WIN32

#include <io.h>
#include <fcntl.h>

#ifndef __GNUC__

#define	R_OK	4
#define	W_OK	2
#define	X_OK	1
#define	F_OK	0

#define O_RDWR		_O_RDWR
#define O_CREAT		_O_CREAT
#define O_EXCL		_O_EXCL
#define O_SYNC		0
#define O_RDONLY	_O_RDONLY
#define O_WRONLY	_O_WRONLY
#define O_TRUNC		_O_TRUNC
#define O_APPEND	_O_APPEND

#endif
#endif

#ifdef SEEK_SET
#define LSEEK_SET	SEEK_SET
#define LSEEK_INCR	SEEK_CUR
#define LSEEK_END	SEEK_END
#else
#define LSEEK_SET	0
#define LSEEK_INCR	1
#define LSEEK_END	2
#endif

/* options for the expand_filename() function */
#define EXPAND_SYNTACTIC 0	/* expand ~ and $, remove redundant . and / */
#define EXPAND_STANDARD	 1	/* SYNTACTIC (proccess) or ABSOLUTE (private) */
#define EXPAND_ABSOLUTE	 2	/* SYNTACTIC + absolute */
#define EXPAND_NORMALISE 3	/* ABSOLUTE + symlinks, capitalisation etc */


/* When to use a timer thread instead of SIGALRM */
#if defined(HAVE_PTHREAD_H) || defined(_WIN32)
#define USE_TIMER_THREAD
#endif


/* Constants for ec_os_err_string() to indicate
 * what kind of error number we want to decode
 */
#define ERRNO_UNIX	0
#define ERRNO_WIN32	1

/* OS-dependent access to thread-specific last error number */

#ifdef _WIN32
#define SetLastOSError(e) SetLastError(e)
#define GetLastOSError() GetLastError()
#else
#define SetLastOSError(e) errno = (e);
#define GetLastOSError() errno
#endif


#ifdef _WIN32
#  include 	<stdlib.h>
#  define MAX_PATH_LEN	_MAX_PATH
#else
#ifdef PATH_IN_LIMITS
#  include 	<limits.h>
#  define MAX_PATH_LEN	PATH_MAX
#else
#  include <sys/param.h>
#  define MAX_PATH_LEN	MAXPATHLEN
#endif
#endif

#ifdef _WIN32
#ifndef __GNUC__
#define S_IFMT	_S_IFMT
#define S_IFDIR	_S_IFDIR
typedef    struct _stat	struct_stat;
#else
typedef    struct stat	struct_stat;
#endif
#else
typedef    struct stat	struct_stat;
#endif

Extern char	ec_version[];
Extern int	clock_hz;
Extern int	ec_use_own_cwd;
Extern int      ec_sigalrm;
Extern int      ec_sigio;

void	ec_os_init(void);
void	ec_os_fini(void);
char *	expand_filename(char *in, char *out, int option);
char *	os_filename(char *in, char *out);
char *	canonical_filename(char *in, char *out);
long	user_time(void);	/* ticks */
int	all_times(double *user, double *system, double *elapsed);
long	ec_unix_time(void); /* seconds */
int	ec_thread_cputime(double*);
char *	ec_date_string(char *buf);
int	ec_gethostname(char *buf, int size);
int	ec_gethostid(char *buf, int size);
int	get_cwd(char *buf, int size);
int	ec_get_cwd(char *buf, int size);
int	ec_set_cwd(char *name);
int	ec_rename(char *, char *);
char *	ec_os_err_string(int err,int grp,char *buf,int size);
char *	ec_env_lookup(char*, char*, int*);

#ifdef HAVE_GETHOSTID
#ifdef GETHOSTID_UNDEF
#    if (SIZEOF_LONG == 8)
Extern int	gethostid();
#    else
Extern long	gethostid();
#    endif
#  endif
#else
Extern long	gethostid();
#endif

#ifndef HAVE_STRERROR
char	*strerror(int);
#endif

#ifdef _WIN32
#ifndef __GNUC__
#define bzero(p,n) ZeroMemory(p,n)
int	putenv(char *);
int	lseek(int, long, int);
int	fstat(int handle, struct_stat *buf);
#endif
int	ec_truncate(int);
int	getpid(void);
int	isatty(int);
int	close(int);
int	read(int, void *, unsigned int);
int	write(int, const void *, unsigned int);
int	pipe(int *);
int	dup(int);
int	dup2(int, int);
int	getpagesize(void);
int	ec_getch_raw(int);
int	ec_putch_raw(int);
#endif

/* Use sigsetjmp if possible */
#if defined(HAVE_DECL_SIGSETJMP) && !HAVE_DECL_SIGSETJMP
#define siglongjmp(jb,r) longjmp(jb,r)
#define sigsetjmp(jb,s) setjmp(jb)
#define sigjmp_buf jmp_buf
#endif


/*
 * Threads and synchronization
 */

#if 1	/* map mt_xxx functions to actual implementations */
#define	mt_mutex_init(m)		ec_mutex_init(m,0)
#define	mt_mutex_init_recursive(m) 	ec_mutex_init(m,1)
#define	mt_mutex_destroy(m)		ec_mutex_destroy(m)
#define	mt_mutex_lock(m)		ec_mutex_lock(m)
#define	mt_mutex_trylock(m)		ec_mutex_trylock(m)
#define	mt_mutex_unlock(m)		ec_mutex_unlock(m)

#else /* disable all locking */
#define	mt_mutex_init(m)		1
#define	mt_mutex_init_recursive(m)	1
#define	mt_mutex_destroy(m)		1
#define	mt_mutex_lock(m)		1
#define	mt_mutex_trylock(m)		1
#define	mt_mutex_unlock(m)		1
#endif


#if _WIN32
int ec_mutex_init(CRITICAL_SECTION*,int recursive);
int ec_mutex_destroy(CRITICAL_SECTION*);
int ec_mutex_lock(CRITICAL_SECTION*);
int ec_mutex_trylock(CRITICAL_SECTION*);
int ec_mutex_unlock(CRITICAL_SECTION*);
int ec_cond_init(CONDITION_VARIABLE*);
int ec_cond_destroy(CONDITION_VARIABLE*);
int ec_cond_signal(CONDITION_VARIABLE*, int all);
int ec_cond_wait(CONDITION_VARIABLE*, CRITICAL_SECTION*, int timeout_ms);

#elif defined(HAVE_PTHREAD_H)
#include <pthread.h>
int ec_mutex_init(pthread_mutex_t*,int recursive);
int ec_mutex_destroy(pthread_mutex_t*);
int ec_mutex_lock(pthread_mutex_t*);
int ec_mutex_trylock(pthread_mutex_t*);
int ec_mutex_unlock(pthread_mutex_t*);
int ec_cond_init(pthread_cond_t*);
int ec_cond_destroy(pthread_cond_t*);
int ec_cond_signal(pthread_cond_t*, int all);
int ec_cond_wait(pthread_cond_t*, pthread_mutex_t*, int timeout_ms);
#else
#define ec_mutex_init(m,r)	1
#define ec_mutex_destroy(m)	1
#define ec_mutex_lock(m)	1
#define ec_mutex_trylock(m)	1
#define ec_mutex_unlock(m)	1
#define ec_cond_init(c)		0
#define ec_cond_destroy(c)	0
#define ec_cond_signal(c,a)	0
#define ec_cond_wait(c,m,t)	0
#endif

int	ec_set_alarm(double, double, void (*)(long), long, double*, double*);
void *	ec_make_thread(void);
int	ec_start_thread(void* thread, int (*)(void*), void* data);
int	ec_thread_stopped(void* thread, int* result);
int	ec_thread_wait(void* thread, int* result, int timeout);
int	ec_thread_terminate(void* thread, int timeout);

int	ec_thread_create(void** os_thread, void*(*fun)(void*), void* arg);
int	ec_thread_detach(void* os_thread);
int	ec_thread_join(void* os_thread);
int	ec_thread_cancel_and_join(void* os_thread);


/* DLLEXP, because these are used in main() */
Extern	DLLEXP	void *	ec_thread_self();
Extern	DLLEXP	void	ec_thread_exit(void*);
Extern	DLLEXP	void	ec_sleep(double);
Extern	DLLEXP	void	ec_bad_exit(char *);


/*
 * Functions that take filename arguments in ECLiPSe pathname syntax
 */
#ifdef _WIN32
int	ec_chdir(char *);
int	ec_access(char *name, int amode);
int	ec_stat(char *name, struct_stat *buf);
int	ec_rmdir(char *name);
int	ec_mkdir(char *name, int);
int	ec_unlink(char *name);
int	ec_open(const char *, int, int);
#else
#define	ec_chdir(A) chdir(A)
#define	ec_access(A,B) access(A,B)
#define	ec_stat(A,B) stat(A,B)
#define	ec_rmdir(A) rmdir(A)
#define	ec_mkdir(A,B) mkdir(A,B)
#define	ec_unlink(A) unlink(A)
#define	ec_open(A,B,C) open(A,B,C)
#endif

#endif
