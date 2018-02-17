/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
static inline unsigned combine2_16(const unsigned oben, const unsigned unten)
{
	unsigned output = unten;
	#ifdef VDSP
		asm("%0 = %0 OR FDEP %1 BY 16:16;" : "+d"(output) : "d"(oben));
	#else
		output |= oben<<16;
	#endif
	return output;
}
