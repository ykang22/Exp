/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

static inline volatile unsigned& DM(const unsigned adr){
	return *reinterpret_cast<volatile unsigned*>(adr);
}

static inline unsigned dm_read(const unsigned adr){
	return DM(adr);
}

static inline unsigned dm_read(const unsigned*const p){
	return *const_cast<volatile unsigned*>(p);
}

static inline void dm_write(const unsigned adr, const unsigned value){
	DM(adr) = value;
}

static inline void dm_write(unsigned*const p, const unsigned value){
	*const_cast<volatile unsigned*>(p) = value;
}

#define Bit_Mask_Write              0x0         /* write to register        */
#define Bit_Mask_Set                0x4         /* set individual bits      */
#define Bit_Mask_Reset              0x8         /* reset individual bits    */
#define Bit_Mask_Toggle             0xC         /* toggle individual bits   */

//#ifndef NODELAY
	#define FPGA_High_Word 0x0800
	#define FPGA_Low_Word  0x1000
//#else
//	#define FPGA_High_Word 0x1
//	#define FPGA_Low_Word  0x2
//#endif
