/**
 * 2005-10-26:	suppressed warning cc0609
 * 2005-11-08:	suppression of warning enabled only for higher versions of compiler
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
template<class T>
inline T lib_min(const T a, const T b){
	return (a<b) ? a : b;
}

#if defined VDSP && defined __2116x__

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(suppress:609)
#endif

#pragma const
template<>
inline float lib_min<float>(const float a, const float b){
	float f;
	asm("%0 = MIN(%1,%2);" : "=F"(f) : "F"(a), "F"(b) );
	return f;
}

#pragma const
template<>
inline unsigned lib_min<unsigned>(const unsigned a, const unsigned b){
	unsigned u;
	asm("%0 = MIN(%1,%2);" : "=d"(u) : "d"(a), "d"(b) );
	return u;
}

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(warning:609)
#endif

#endif
