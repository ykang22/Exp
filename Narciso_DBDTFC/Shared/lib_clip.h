/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#include"lib_min.h"
#include"lib_max.h"

#ifdef VDSP
	#pragma const
#endif
template<class T>
inline T lib_clip(const T t, const T t_min, const T t_max){
	return lib_min(lib_max(t,t_min),t_max);
}
