/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#include<sysreg.h>
	#include<def21161.h>
	//from def21161:  #define IRPTEN  BIT_12
#else
	#include"assert.h"
#endif

extern int atomic_depth;

inline static void atomic_on(void){
	if(atomic_depth++==0){
		#ifdef VDSP
			sysreg_bit_clr(sysreg_MODE1,IRPTEN);
		#endif
	}
}

inline static void atomic_off(void){
	if(--atomic_depth==0){
		#ifdef VDSP
			sysreg_bit_set(sysreg_MODE1,IRPTEN);
		#endif
	}
	#ifdef WIN32
		assert(atomic_depth>=0);
	#endif
}
