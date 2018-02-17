/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
static inline unsigned trunc(const float f)
{
#ifdef VDSP
	unsigned u;
	asm("%0 = TRUNC %1;" : "=d"(u) : "F"(f));
	return u;
#else
	return static_cast<unsigned>(f);
#endif
}

#ifdef VDSP
	#pragma const
#endif
static inline unsigned trunc_by(const float f, const int exp_offs)
{
#ifdef VDSP
	unsigned u;
	asm("%0 = TRUNC %1 BY %2;" : "=d"(u) : "F"(f), "d"(exp_offs));
	return u;
#else
	return static_cast<unsigned>(f*static_cast<float>(1u<<exp_offs));
#endif
}
