/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
template<class T>
static inline void swap(T &a, T &b){
	const T x = a;
	a = b;
	b = x;
}

