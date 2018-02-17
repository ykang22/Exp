/**
 * 2005-10-26:	added #include<cassert>
 **/

#pragma once
#include"always.h"

#ifdef DEBUG
	#ifdef VDSP
		#include"error.h"
		#define assert_complete(x, errnum, errmsg, errfile, errln) if( ! (x)) Error(errnum, errmsg, errfile, errln)
		#define assert(x) assert_complete(x, ERR_ass, "Assertion failed: " #x, __FILE__, __LINE__ )
	#else
		#include<cassert>
		#define assert_complete(x, errnum, errmsg, errfile, errln) assert(false&& errnum && errmsg)
	#endif

	#define assert_message(x, errnum, errmsg) assert_complete(x, errnum, errmsg, __FILE__, __LINE__)
	#define assert_false(errnum, errmsg) assert_complete(false, errnum, errmsg, __FILE__, __LINE__)
	#define assert_min(x, min_incl, errnum, errmsg) assert_complete(((x)>=min_incl), errnum, errmsg, __FILE__, __LINE__)
	#define assert_max(x, max_excl, errnum, errmsg) assert_complete(((x)<max_excl), errnum, errmsg, __FILE__, __LINE__)
	#define assert_range(x, min_incl, max_excl, errnum, errmsg) assert_complete(((x)>=min_incl)&&((x)<max_excl), errnum, errmsg, __FILE__, __LINE__)
#else
	#ifdef VDSP
		#define assert(x)
	#else
		#include<cassert>
	#endif
	#define assert_complete(x, errnum, errmsg)

	#define assert_message(x, errnum, errmsg)
	#define assert_false(errnum, errmsg)
	#define assert_min(x, min_incl, errnum, errmsg)
	#define assert_max(x, max_excl, errnum, errmsg)
	#define assert_range(x, min_incl, max_excl, errnum, errmsg)
#endif
