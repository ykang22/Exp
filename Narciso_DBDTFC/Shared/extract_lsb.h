/**
 * 2005-03-17:	copied from extract_bit.h
 **/

#pragma once
#include"always.h"

#include"extract_bit.h"
#include"bitmask_to_bitpos.h"


#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline bool extract_lsb(const unsigned input){
	return extract_bit<bitmask_to_ls_bitpos(mask)>(input);
}

#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline bool dynamic_extract_lsb(const unsigned input){
	return dynamic_extract_bit<bitmask_to_ls_bitpos(mask)>(input);
}

