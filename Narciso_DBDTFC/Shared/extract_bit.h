/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#include"extract.h"

#ifdef VDSP
	#pragma const
#endif
template<unsigned pos>
inline bool extract_bit(const unsigned input){
	const unsigned result = extract<pos,pos+1>(input);
	return *reinterpret_cast<const bool*>(&result);
}

#ifdef VDSP
	#pragma const
#endif
inline bool dynamic_extract_bit(const unsigned input, const unsigned pos){
	const unsigned result = (input>>pos)&0x1u;
	return *reinterpret_cast<const bool*>(&result);
}

