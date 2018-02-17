/**
 * 2005-03-17:	copied from extract_msb.h
 **/

#pragma once
#include"always.h"

#include"extract.h"
#include"bitmask_to_bitpos.h"

/**
 * Extracts all bits between the highest and the lowest (inclusive) bits set in the mask.
 * Example:
 *   extract_mask<0x000F0FF0>(u)   is equivalent to
 *   extract<4,20>(u)
 **/
#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline unsigned extract_mask(const unsigned input){
	return extract<bitmask_to_ls_bitpos(mask),bitmask_to_ms_bitpos(mask)+1>(input);
}

#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline bool dynamic_extract_mask(const unsigned input){
	return dynamic_extract<bitmask_to_ls_bitpos(mask),bitmask_to_ms_bitpos(mask)+1>(input);
}

