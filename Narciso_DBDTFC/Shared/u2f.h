/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
inline float u2f(const unsigned u){
	return *reinterpret_cast<const float*>(&u);
}
