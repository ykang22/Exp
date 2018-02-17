/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

/**
 * type of argument which is passed to an interrupt function by the system
 * WARNING: this type will change with a new interrupt management!
 **/
#ifdef VDSP
	typedef const int irq_param;
#else
	typedef void irq_param;
#endif

/**
 * pointer to a function which can be used in interrupts
 **/
typedef void (*irq_func)(irq_param);
