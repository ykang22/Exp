/**
 * 2005-03-31:	corrected
 **/

#pragma once
#include"always.h"

template<class T>
static inline void copy(const T &source, T &drain){
	const unsigned*s = reinterpret_cast<const unsigned*>(&source);
	      unsigned*d = reinterpret_cast<      unsigned*>(&drain);
	for(unsigned i=0; i<sizeof(T)/sizeof(unsigned); ++i){
		*d++ = *s++;
	}
};

