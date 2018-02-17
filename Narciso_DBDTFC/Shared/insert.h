/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#include"deposit.h"

#ifdef VDSP
	#pragma const
#endif
template<unsigned begin,unsigned end>
inline unsigned insert(const unsigned input, const unsigned seed){
	const unsigned u = input & ~((0xFFFFFFFF>>(32u-end+begin))<<begin);
	return u | deposit<begin,end>(seed);;
}

#ifdef VDSP
	#pragma const
#endif
inline unsigned dynamic_insert(const unsigned input, const unsigned seed, const unsigned begin, const unsigned end){
	const unsigned u = input & ~((0xFFFFFFFF>>(32u-end+begin))<<begin);
	return u | dynamic_deposit(seed,begin,end);
}
