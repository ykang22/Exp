/**
 * 2005-10-25:	added EXTERN_CPP
 **/

#pragma once

/* VDSP detection */
#ifndef WIN32
	#define VDSP
#endif

/* LIB detection */
#ifdef VDSP
	#define EXTERN extern
	#define EXTERN_CPP extern
	#define INLINE inline
#else
	#ifdef XCM2000_EXPORTS
		#define LIB
		#define EXTERN extern "C" __declspec(dllexport)
		#define EXTERN_CPP extern __declspec(dllexport)
		#define INLINE inline
	#else
		#define EXTERN extern "C" __declspec(dllimport)
		#define EXTERN_CPP extern __declspec(dllimport)
		#define INLINE extern "C" __declspec(dllexport)
	#endif
#endif

/* DEBUG detection */
#ifdef _DEBUG
	#define DEBUG
#endif

#if defined NODELAY && !defined DEBUG
	#define DEBUG
#endif
