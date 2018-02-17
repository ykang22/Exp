/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"



#define DSP_CLOCK 100e6f
#define DSP_CYCLE_TIME (1.f/DSP_CLOCK)
#define FPGA_CLOCK 50e6f
#define FPGA_CYCLE_TIME (1.f/FPGA_CLOCK)

#ifdef VDSP
	#pragma const
#endif
inline float dsp_clock(){
	return DSP_CLOCK;
}

#ifdef VDSP
	#pragma const
#endif
inline float dsp_cycle_time(){
	return DSP_CYCLE_TIME;
}

#ifdef VDSP
	#pragma const
#endif
inline float fpga_clock(){
	return FPGA_CLOCK;
}

#ifdef VDSP
	#pragma const
#endif
inline float fpga_cycle_time(){
	return FPGA_CYCLE_TIME;
}

#undef DSP_CLOCK
#undef DSP_CYCLE_TIME
#undef FPGA_CLOCK
#undef FPGA_CYCLE_TIME
