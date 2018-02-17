/**
 * add_version_notes_here
 **/

#pragma once
#include"shared/always.h"

#include"shared/irq_typedefs.h"

////////////////////////////////////////////////////////////////////////////////
// Inititialization
////////////////////////////////////////////////////////////////////////////////
#ifdef S_DSP
	EXTERN void S_DSP_init(void);
#else
	EXTERN void C_DSP_init(void);
#endif

// sets SYSCON flags and WAIT modes to defaults  // is called in init.h by main()
inline void DSP_init(void)
{
	#ifdef S_DSP	
		S_DSP_init();
	#else		
		C_DSP_init();
	#endif
}

////////////////////////////////////////////////////////////////////////////////
// Delay
////////////////////////////////////////////////////////////////////////////////

EXTERN void delay(const float periode);
	// executes 'NOP' for the time specified [s]

////////////////////////////////////////////////////////////////////////////////
// SHARC's internal Timer
////////////////////////////////////////////////////////////////////////////////

EXTERN void timer0_set_periode_cycles(const unsigned periode);  // Change the SHARC's internal Timer's interval time [cycles]

EXTERN void timer0_init(irq_func func, const float periode, const bool high_priority = true);  // Init SHARC's internal Timer with user function and periode [s] - Does not start the Timer
	// high_priority, if false, signalizes priority lower than IRQ2 to IRQ0, otherwise higher

EXTERN void timer0_start();  // Enable SHARC's internal Timer

EXTERN void timer0_stop();  // Disable SHARC's internal Timer

EXTERN void timer0_set_periode(const float periode);  // Change the SHARC's internal Timer's interval time [s]

EXTERN void timer0_set_freq(const float freq);  // Change the SHARC's internal Timer's frequency [Hz]

EXTERN void timer0_set_irqfunc(irq_func func);

