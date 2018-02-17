/**
 * add_version_notes_here
 **/

#pragma once
#include"../shared/always.h"

enum ERR{
	ERR_ass = 0x81,
	ERR_msc_ass_serial,
	ERR_msc_ass_watchdog,
	ERR_msc_ass_timer,
	ERR_irq_ass,
	ERR_adda_ass = 0x82,
	ERR_adda_ass_channel = 0x83,
	ERR_adda_ass_fpga = 0x84,
	ERR_adda_ass_gain = 0x85,
	ERR_adda_ass_fastconversion = 0x87,
	ERR_adda_ass_pair = 0x88,
	ERR_adda_ass_converter = 0x89,
	ERR_pwm_ass = 0x41,
	ERR_pwm_ass_channel = 0x42,
	ERR_pwm_ass_block = 0x43,
	ERR_pwm_ass_fpga = 0x44,
	ERR_pwm_ass_dutycycle = 0x45,
	ERR_pwm_ass_deadtime = 0x46,
	ERR_dio_ass = 0x47,
	ERR_serial_prompt_ass,
	ERR_eeprom_ass
};

////////////////////////////////////////////////////////////////////////////////
// Display Error
////////////////////////////////////////////////////////////////////////////////

unsigned Error(const unsigned error_num, const char error_msg[], const char error_file[] = 0, const unsigned error_line = 0);
	// Writes the string to the RS232
	// in DEBUG only:
	// May be used to signalize any error on the LEDs
	// by blinking the error code half-secondly
	// waites for button_down() then button_up()
