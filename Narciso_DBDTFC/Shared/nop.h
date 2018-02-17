/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#define nop asm volatile("nop;")
	#define noop asm volatile("nop;nop;")
	#define nooop asm volatile("nop;nop;nop;nop;")
	#define noooop asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;")
#else
	#define nop
	#define noop
	#define nooop
	#define noooop
#endif
