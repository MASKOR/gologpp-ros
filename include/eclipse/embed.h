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
 * Copyright (C) 1997-2006 Cisco Systems, Inc.  All Rights Reserved.
 * 
 * Contributor(s): 
 * 
 * END LICENSE BLOCK */

/*
 * ECLiPSe LIBRARY MODULE
 *
 * $Id: embed.h,v 1.20 2017/09/01 03:05:09 jschimpf Exp $
 *
 *
 * IDENTIFICATION:	embed.h
 *
 * AUTHOR:		Joachim Schimpf
 * AUTHOR:		Stefano Novello
 *
 * CONTENTS:		name/arity
 *
 * DESCRIPTION:
 *			External embedding interface and safe variables
 */


enum {
	EC_OPTION_MAPFILE		=0,
	EC_OPTION_PARALLEL_WORKER	=1,
	EC_OPTION_ARGC			=2,
	EC_OPTION_ARGV			=3,
	EC_OPTION_LOCALSIZE		=4,
	EC_OPTION_GLOBALSIZE		=5,
	EC_OPTION_PRIVATESIZE		=6,
	EC_OPTION_SHAREDSIZE		=7,
	EC_OPTION_PANIC			=8,
	EC_OPTION_ALLOCATION		=9,
	EC_OPTION_DEFAULT_MODULE	=10,
	EC_OPTION_ECLIPSEDIR		=11,
	EC_OPTION_IO			=12,
	EC_OPTION_INIT			=13,
	EC_OPTION_DEBUG_LEVEL		=14,
	EC_OPTION_CWD_SEPARATE		=15,
	EC_OPTION_DEFAULT_LANGUAGE	=16,
	EC_OPTION_WITH_PROFILER		=17
};

/*
 * Data
 */

Extern DLLEXP t_eclipse_data		ec_;
Extern DLLEXP const t_ext_type		ec_xt_double_arr;
Extern DLLEXP const t_ext_type		ec_xt_long_arr;
Extern DLLEXP const t_ext_type		ec_xt_char_arr;


/*
 * Initialisation options
 */
Extern DLLEXP int	Winapi	ec_set_option_int(int, int);
Extern DLLEXP int	Winapi	ec_set_option_long(int, word);
Extern DLLEXP int	Winapi	ecl_set_option_long(t_eclipse_options*, int, word);
Extern DLLEXP int	Winapi	ec_set_option_ptr(int, void*);
Extern DLLEXP int	Winapi	ecl_set_option_ptr(t_eclipse_options*, int, void*);

/* set the context module for resuming with goals */
Extern DLLEXP int	Winapi	ecl_set_context_module(ec_eng_t*,dident);

/*
 * Create and destroy an eclipse engine
 */
Extern DLLEXP int	Winapi	ec_init(void);
Extern DLLEXP int	Winapi	ecl_init(t_eclipse_options*,ec_eng_t**);
Extern DLLEXP int	Winapi	ec_cleanup(void);

Extern DLLEXP int	Winapi	ecl_engine_create(t_eclipse_options*,ec_eng_t*,ec_eng_t**);
Extern DLLEXP int	Winapi	ecl_acquire_engine(ec_eng_t*);
Extern DLLEXP void	Winapi	ecl_relinquish_engine(ec_eng_t*);
Extern DLLEXP int	Winapi	ecl_relinquish_engine_opt(ec_eng_t*, int allow_exit);
Extern DLLEXP int	Winapi	ecl_request_exit(ec_eng_t*,int);
Extern DLLEXP int	Winapi	ecl_request_throw(ec_eng_t*,pword);

/*
 * Restart an eclipse engine that has yielded
 */
Extern DLLEXP int	Winapi	ecl_post_goal(ec_eng_t*,const pword);

Extern DLLEXP int	Winapi	ecl_post_string(ec_eng_t*,const char *);

Extern DLLEXP int	Winapi	ecl_post_exdr(ec_eng_t*,int, const char *);

Extern DLLEXP int	Winapi	ecl_resume(ec_eng_t*);

Extern DLLEXP int	Winapi	ecl_resume1(ec_eng_t*,ec_ref);

Extern DLLEXP int	Winapi	ecl_resume2(ec_eng_t*,const pword,ec_ref);

Extern DLLEXP int	Winapi	ecl_resume_goal(ec_eng_t*,const pword,const pword,ec_ref,int);

Extern DLLEXP int	Winapi	ecl_resume_long(ec_eng_t*,long *);

Extern DLLEXP int	Winapi	ecl_resume_async(ec_eng_t*);
Extern DLLEXP int	Winapi	ecl_resume_async1(ec_eng_t*,const pword,const pword);

Extern DLLEXP int	Winapi	ec_running(void);
Extern DLLEXP int	Winapi	ecl_resume_status(ec_eng_t*);
Extern DLLEXP int	Winapi	ecl_resume_status_long(ec_eng_t*,long *);
Extern DLLEXP int	Winapi	ecl_wait_resume_status_long(ec_eng_t*,long *, int);

Extern DLLEXP int	Winapi	ecl_join_acquire(ec_eng_t*, int);


/*
 * Send events to running engine and handle them
 * (Note that events can also be raised by queues)
 */
Extern DLLEXP int	Winapi	ecl_post_event(ec_eng_t*,pword);
Extern DLLEXP int	Winapi	ecl_post_event_unique(ec_eng_t*,pword);
Extern DLLEXP int	Winapi	ecl_post_throw(ec_eng_t *from_eng, ec_eng_t *ec_eng, pword ball);

Extern DLLEXP int	Winapi	ecl_post_event_string(ec_eng_t*,const char *);

Extern DLLEXP int	Winapi	ecl_handle_events(ec_eng_t*);

Extern DLLEXP int	Winapi	ecl_handle_events_long(ec_eng_t*,long*);

/*
 * Choicepoints
 */
Extern DLLEXP void	Winapi	ec_cut_to_chp(ec_ref);

/*
 * construct eclipse terms
 */
Extern DLLEXP pword	Winapi	ecl_string(ec_eng_t*,const char*);

Extern DLLEXP pword	Winapi	ecl_length_string(ec_eng_t*,int, const char*);

Extern DLLEXP pword	Winapi	ec_atom(const dident);

Extern DLLEXP pword	Winapi	ec_long(const long);
#ifdef HAVE_LONG_LONG
Extern DLLEXP pword	Winapi	ecl_long_long(ec_eng_t*,const long long);
#endif
Extern DLLEXP pword	Winapi	ecl_double(ec_eng_t*,const double);
Extern DLLEXP pword		ecl_term(ec_eng_t*,dident, ... /*pwords*/);
			/* can't use Winapi with varargs! */
Extern DLLEXP pword	Winapi	ecl_term_array(ec_eng_t*,const dident,const pword[]);
Extern DLLEXP pword	Winapi	ecl_list(ec_eng_t*,const pword,const pword);
Extern DLLEXP pword	Winapi	ecl_listofrefs(ec_eng_t*,ec_refs);
Extern DLLEXP pword	Winapi	ecl_listofdouble(ec_eng_t*,int, const double*);
Extern DLLEXP pword	Winapi	ecl_listoflong(ec_eng_t*,int, const long*);
Extern DLLEXP pword	Winapi	ecl_listofchar(ec_eng_t*,int, const char*);
Extern DLLEXP pword	Winapi	ecl_arrayofdouble(ec_eng_t*,int, const double*);
Extern DLLEXP pword	Winapi	ecl_matrixofdouble(ec_eng_t*,int, int, const double*);

Extern DLLEXP pword	Winapi	ecl_handle(ec_eng_t*,const t_ext_type*,const t_ext_ptr);

Extern DLLEXP pword	Winapi	ec_nil(void);
Extern DLLEXP dident	Winapi	ec_did(const char *,const int);

Extern DLLEXP pword	Winapi	ecl_newvar(ec_eng_t*);

/* for consistency (engine argument not needed) */
#define ecl_nil(ec_eng) ec_nil()
#define ecl_long(ec_eng,l) ec_long(l)
#define ecl_atom(ec_eng,d) ec_atom(d)


/*
 * inspect eclipse terms
 */
Extern DLLEXP int	Winapi	ec_get_string(const pword,char**);
Extern DLLEXP int	Winapi	ec_get_string_length(const pword,char**,long*);
Extern DLLEXP int	Winapi	ec_get_atom(const pword,dident*);
Extern DLLEXP int	Winapi	ec_get_long(const pword,long*);
#ifdef HAVE_LONG_LONG
Extern DLLEXP int	Winapi	ec_get_long_long(const pword,long long*);
#endif
Extern DLLEXP int	Winapi	ec_get_double(const pword,double*);
Extern DLLEXP int	Winapi	ec_get_nil(const pword);
Extern DLLEXP int	Winapi	ec_get_list(const pword,pword*,pword*);
Extern DLLEXP int	Winapi	ec_get_functor(const pword,dident*);
Extern DLLEXP int	Winapi	ec_arity(const pword);
Extern DLLEXP int	Winapi	ec_get_arg(const int,pword,pword*);
Extern DLLEXP int	Winapi	ec_get_handle(const pword,const t_ext_type*,t_ext_ptr*);
Extern DLLEXP int	Winapi	ec_is_var(const pword);

#define DidName(d)	((char *)(((dident)(d))->string + 1))
#define DidArity(d)	(((dident)(d))->arity)

/*
 * eclipse refs hold registered references to eclipse terms
 * which survive while the engine is running
 */
Extern DLLEXP ec_refs	Winapi	ecl_refs_create(ec_eng_t*,int,const pword);
Extern DLLEXP ec_refs	Winapi	ecl_refs_create_newvars(ec_eng_t*,int);
Extern DLLEXP ec_refs	Winapi	ec_refs_copy(ec_refs);
Extern DLLEXP void	Winapi	ec_refs_destroy(ec_refs);
Extern DLLEXP void	Winapi	ec_refs_set(ec_refs,int,const pword);
Extern DLLEXP pword	Winapi	ec_refs_get(const ec_refs,int);
Extern DLLEXP int	Winapi	ec_refs_size(const ec_refs);
Extern DLLEXP ec_eng_t*	Winapi	ec_refs_get_engine(const ec_refs);

Extern DLLEXP ec_ref	Winapi	ecl_ref_create(ec_eng_t*,pword);
Extern DLLEXP ec_ref	Winapi	ecl_ref_create_newvar(ec_eng_t*);
Extern DLLEXP ec_ref	Winapi	ec_ref_copy(ec_ref);
Extern DLLEXP void	Winapi	ec_ref_destroy(ec_ref);
Extern DLLEXP void	Winapi	ec_ref_set(ec_ref,const pword);
Extern DLLEXP pword	Winapi	ec_ref_get(const ec_ref);
Extern DLLEXP ec_eng_t*	Winapi	ec_ref_get_engine(const ec_ref);


/*
 * String-based interface
 */

Extern DLLEXP int	Winapi	ecl_exec_string(ec_eng_t*,char*,ec_ref);

Extern DLLEXP int	Winapi	ec_var_lookup(ec_ref,char*,pword*);

/*
 * External function interface
 */

Extern DLLEXP pword	Winapi	ecl_arg(ec_eng_t*,int);

Extern DLLEXP int	Winapi	ecl_unify(ec_eng_t*,pword,pword);

Extern DLLEXP int	Winapi	ecl_unify_arg(ec_eng_t*,int,pword);

Extern DLLEXP int	Winapi	ec_compare(pword,pword);
Extern DLLEXP int	Winapi	ec_visible_procedure(dident,pword,void**);

Extern DLLEXP int	Winapi	ecl_make_suspension(ec_eng_t*,pword,int,void*,pword*);

Extern DLLEXP int	Winapi	ecl_schedule_suspensions(ec_eng_t*,pword,int);

Extern DLLEXP int	Winapi	ec_free_handle(const pword, const t_ext_type*);
Extern DLLEXP int		ec_external(dident,int (*) (Dots),dident);

/*
 * Stream I/O
 */

Extern DLLEXP int	Winapi	ec_get_stream(const pword,stream_id*);
Extern DLLEXP void	Winapi	ec_release_stream(stream_id);

Extern DLLEXP int	Winapi	ec_stream_nr(const char *name);
Extern DLLEXP int	Winapi	ec_queue_write(int stream, char *data, int size);
Extern DLLEXP int	Winapi	ec_queue_read(int stream, char *data, int size);
Extern DLLEXP int	Winapi	ec_queue_avail(int stream);
Extern DLLEXP void	Winapi	ec_double_xdr(double * d, char * dest);
Extern DLLEXP void	Winapi	ec_int32_xdr(int32_t * l, char * dest);
Extern DLLEXP void	Winapi	ec_xdr_int32(char * buf , int32_t * l);
Extern DLLEXP void	Winapi	ec_xdr_double(char * buf , double * d);

/*
 * Error handling
 */

Extern DLLEXP char *	Winapi	ec_error_string(int);
Extern DLLEXP void	Winapi	ec_make_error_message(int err, char *where, char *buf, int size);
Extern DLLEXP void		ec_panic(const char* what, const char* where); /* no Winapi */


/*
 * Backward compatibility definitions without engine argument.
 * These macros rely on a variable called 'ec_eng' being visible,
 * which refers to the engine to be used.
 */

#ifdef USES_NO_ENGINE_HANDLE
#define ec_eng (&ec_.m)
#endif

#ifdef USES_NO_ENGINE_HANDLE
#define ec_term(dident,...)	ecl_term(ec_eng,dident,__VA_ARGS__)

#define ec_post_goal(g)		ecl_post_goal(ec_eng,g)
#define ec_post_string(s)	ecl_post_string(ec_eng,s)
#define ec_post_exdr(l,s)	ecl_post_exdr(ec_eng,l,s)
#define ec_resume()		ecl_resume(ec_eng)
#define ec_resume1(r)		ecl_resume1(ec_eng,r)
#define ec_resume2(p,r)		ecl_resume2(ec_eng,p,r)
#define ec_resume_long(i)	ecl_resume_long(ec_eng,i)
#define ec_resume_async()	ecl_resume_async(ec_eng)
#define ec_resume_async1(p)	ecl_resume_async1(ec_eng,p)
#define ec_resume_status(i,t)	ecl_resume_status(ec_eng)
#define ec_resume_status_long(i,t)	ecl_resume_status_long(ec_eng,i)
#define ec_wait_resume_status_long(i,t)	ecl_wait_resume_status_long(ec_eng,i,t)
#define ec_post_event(e)	ecl_post_event(ec_eng,e)
#define ec_post_event_string(s)	ecl_post_event_string(ec_eng,s)
#define ec_handle_events(pl)	ecl_handle_events_long(ec_eng,pl)

#define ec_exec_string(s,r)	ecl_exec_string(ec_eng,s,r)

#define ec_string(s)		ecl_string(ec_eng, s)
#define ec_length_string(l, s)	ecl_length_string(ec_eng, l, s)
#define ec_long_long(l)		ecl_long_long(ec_eng, l)
#define ec_double(d)		ecl_double(ec_eng,d)
#define ec_handle(c,d)		ecl_handle(ec_eng,c,d)
#define ec_newvar()		ecl_newvar(ec_eng)

#define ec_refs_create(i,p)	ecl_refs_create(ec_eng,i,p)
#define ec_refs_create_newvars(i)	ecl_refs_create_newvars(ec_eng,i)
#define ec_ref_create(p)	ecl_ref_create(ec_eng,p)
#define ec_ref_create_newvar()	ecl_ref_create_newvar(ec_eng)

#define ec_term_array(d,a)	ecl_term_array(ec_eng,d,a)
#define ec_list(h,t)		ecl_list(ec_eng,h,t)
#define ec_listofrefs(r)	ecl_listofrefs(ec_eng,r)
#define ec_listofdouble(i,pd)	ecl_listofdouble(ec_eng,i,pd)
#define ec_listoflong(i,pl)	ecl_listoflong(ec_eng,i,pl)
#define ec_listofchar(i,pc)	ecl_listofchar(ec_eng,i,pc)
#define ec_arrayofdouble(i,pd)	ecl_arrayofdouble(ec_eng,i,pd)
#define ec_matrixofdouble(i,j,pd)	ecl_matrixofdouble(ec_eng,i,j,pd)

#define ec_arg(i)		ecl_arg(ec_eng,i)
#define ec_unify(p1,p2)		ecl_unify(ec_eng,p1,p2)
#define ec_unify_arg(i,p)	ecl_unify_arg(ec_eng,i,p)
#define	ec_make_suspension(g,i,p,s)	ecl_make_suspension(ec_eng,g,i,p,s)
#define ec_schedule_suspensions(p,i)	ecl_schedule_suspensions(ec_eng,p,i)
#endif



/*
 * The following is NOT (yet) part of the official embedding interface!
 */

#define current_input_	ec_.current_input
#define current_output_	ec_.current_output
#define current_err_	ec_.current_error
#define warning_output_	ec_.current_warning_output
#define log_output_	ec_.current_log_output
#define user_input_	ec_.user_input
#define user_output_	ec_.user_output
#define user_err_	ec_.user_error
#define null_		ec_.null_stream


Extern DLLEXP stream_id Winapi	ec_stream_id(int);
Extern DLLEXP	int		ec_outf(stream_id, const char*, int);
Extern DLLEXP	int		ec_outfc(stream_id, int);
Extern DLLEXP	int		ec_outfs(stream_id, const char*);
Extern DLLEXP	int		ec_flush(stream_id);
Extern DLLEXP	int		p_fprintf(stream_id nst, const char *fmt, ...);
Extern DLLEXP	int		ec_printff(stream_id nst, const char *fmt, ...);
Extern DLLEXP	int		ec_newline(stream_id);
Extern DLLEXP	int		ec_print_msg(stream_id nst, const char *fmt, int);


/* ec_untrail_undo() used in gfd.cpp */

/* The context in which an undo function is being called */
#define UNDO_FAIL		0	/* untrail during fail */
#define UNDO_GC			1	/* untrail during gc */

/* Type of trailed data */
#define TRAILED_PWORD		0x0
#define TRAILED_REF		0x4
#define TRAILED_WORD32		0x8
#define TRAILED_COMP		0xc

#ifdef USES_NO_ENGINE_HANDLE
#define ec_trail_undo(f,pi,ps,pd,s,t) ecl_trail_undo(ec_eng,f,pi,ps,pd,s,t)
#endif
Extern DLLEXP void		ecl_trail_undo(ec_eng_t*,void (*)(pword*,word*,int,int,ec_eng_t*), pword*, pword*, word*, int, int);


/* schedule_cut_fail_action() used in Oci */
#ifdef USES_NO_ENGINE_HANDLE
#define	schedule_cut_fail_action(f,v,t) ecl_schedule_cut_fail_action(ec_eng, (void(*)(value,type,ec_eng_t*))f, v, t)
#endif
Extern	DLLEXP	void	ecl_schedule_cut_fail_action(ec_eng_t*, void (*)(value,type,ec_eng_t*), value, type);


#ifndef EC_EMBED

Extern t_eclipse_options	ec_options;
Extern char			*ec_eclipse_home;


#ifdef USES_NO_ENGINE_HANDLE
#define ec_assign(p,v,t) ecl_assign(ec_eng,p,v,t)
#endif
Extern	DLLEXP	int 	ecl_assign(ec_eng_t*, pword*, value, type);

Extern DLLEXP int		ecl_request(ec_eng_t*,int);

#if 0
Extern DLLEXP void		ec_trail_pwords(pword*, int, int);
#endif
Extern DLLEXP int		ec_unify_(ec_eng_t*,value,type,value,type,pword**);
Extern DLLEXP int		ec_remember(ec_eng_t*,int,value,type);
Extern DLLEXP void		cut_external(ec_eng_t*);

Extern 		void		delayed_break(void);
Extern 		int		final_overflow(ec_eng_t*);
Extern DLLEXP void		global_ov(ec_eng_t*);
Extern DLLEXP void		trail_ov(ec_eng_t*);

Extern 		volatile int	it_disabled_, delayed_it_;

Extern DLLEXP	int		p_handle_free(value v_handle, type t_handle, ec_eng_t *);
Extern 		int		p_handle_free_on_cut(value v_handle, type t_handle, ec_eng_t *);

Extern DLLEXP 	pword * 	term_to_dbformat(ec_eng_t*, pword*, dident);
Extern DLLEXP 	pword * 	dbformat_to_term(ec_eng_t*, char*, dident, type);

Extern	DLLEXP	int		meta_index(dident);

Extern	void	ec_cleanup_unlock(void *);

Extern	void	ec_ref_set_safe(ec_ref variable, const pword w);

#endif /* !EC_EMBED */

