/**
 * 2005-10-26:	suppressed warning cc0609
 * 2005-11-08:	removed release-optimization
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
inline unsigned dynamic_deposit(const unsigned input, const unsigned begin, const unsigned end){
	return (input & (0xFFFFFFFF>>(32u-end+begin)))<<begin;
}

#ifdef VDSP
	#pragma const
#endif
template<unsigned begin,unsigned end>
inline unsigned deposit(const unsigned input){
	return (input & (0xFFFFFFFF>>(32u-end+begin)))<<begin;
}
