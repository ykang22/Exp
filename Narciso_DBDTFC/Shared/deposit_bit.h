/**
 * 2005-07-11:	no more write to reference, but output value
 *            	Attention!   assert( (input==true) <=> (input&0x1==0x1) ); !!!
 **/

#pragma once
#include"always.h"

#include"deposit.h"

#ifdef VDSP
	#pragma const
#endif
template<unsigned pos>
inline unsigned deposit_bit(const bool input){
	return deposit<pos,pos+1>(input);
}

