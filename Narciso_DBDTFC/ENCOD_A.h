// $Id: ENCOD_A.h 75 2007-08-29 08:29:38Z vb_mf $
/**
 * 2005-05-25:	created from ANALG_N
 **/
#pragma once
#ifndef ENCOD_A_H
#define ENCOD_A_H

#include <shared/always.h>
#include <shared/blocks.h>

#ifdef SHOW_HARDWARE_INCLUDE
	#warning Block ENCOD_A now available.
#endif

#include <ENCOD/ENCOD001.h>
typedef encod001<BLOCK_A> T_ENCOD_A;

extern T_ENCOD_A ENCOD_A;

#ifdef INIT_ONCE
	#ifndef VDSP
		T_ENCOD_A ENCOD_A(0);
	#endif
#endif

#endif // ENCOD_A_H
