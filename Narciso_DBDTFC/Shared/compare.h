/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

template<class T>
static inline bool compare(const T &r0, const T &r1){
	const unsigned*p0 = reinterpret_cast<const unsigned*>(&r0);
	const unsigned*p1 = reinterpret_cast<const unsigned*>(&r1);
	for(unsigned i=0; i<sizeof(T)/sizeof(unsigned); ++i){
		if(*p0++ != *p1++)
			return false;
	}
	return true;
};

