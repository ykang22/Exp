/**
 * 2005-03-17:	copied from extract_msb.h
 **/

#pragma once
#include"always.h"

#include"deposit.h"
#include"bitmask_to_bitpos.h"

/**
 * Deposits the lowest input-bits in between the highest and the lowest (inclusive)
 * bits set in the mask.
 * Example:
 *   deposit_mask<0x000F0FF0>(u)   is equivalent to
 *   deposit<4,20>(u)
 **/
#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline unsigned deposit_mask(const unsigned input){
	return deposit<bitmask_to_ls_bitpos(mask),bitmask_to_ms_bitpos(mask)+1>(input);
}

#ifdef VDSP
	#pragma const
#endif
template<unsigned mask>
inline bool dynamic_deposit_mask(const unsigned input){
	return dynamic_deposit<bitmask_to_ls_bitpos(mask),bitmask_to_ms_bitpos(mask)+1>(input);
}

