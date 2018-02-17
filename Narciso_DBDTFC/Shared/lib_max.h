/**
 * 2005-11-08:	suppression of warning 609 enabled, but only for higher versions of compiler
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
template<class T>
static inline T lib_max(const T a, const T b){
	return (a>b) ? a : b;
}

#if defined VDSP && defined __2116x__

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(suppress:609)
#endif

#pragma const
template<>
inline float lib_max<float>(const float a, const float b){
	float f;
	asm("%0 = MAX(%1,%2);" : "=F"(f) : "F"(a), "F"(b) );
	return f;
}

#pragma const
template<>
inline unsigned lib_max<unsigned>(const unsigned a, const unsigned b){
	unsigned u;
	asm("%0 = MAX(%1,%2);" : "=d"(u) : "d"(a), "d"(b) );
	return u;
}

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(warning:609)
#endif

#endif

