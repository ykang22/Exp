/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
inline unsigned dynamic_extract(unsigned begin,unsigned end,const unsigned input){
	return (input>>begin) & (0xFFFFFFFF>>(32u-end+begin));
}
