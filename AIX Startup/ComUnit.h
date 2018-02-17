// $Id: ComUnit.h 40 2007-01-19 16:45:10Z vb_mf $
#pragma once
#ifndef COMUNIT_H
#define COMUNIT_H

#include <shared/always.h>

/********************************************************************************
 * 	Constants
 ********************************************************************************/

// Display

#define DSPL_DD_SET_ADDR	0x180
#define DSPL_CG_SET_ADDR	0x140

#define DSPL_CLEAR		0x101
#define DSPL_HOME		0x101

#define DSPL_RS			0x100

#define DSPL_FUNC_SET		0x120
#define DSPL_DL			0x010
#define DSPL_N			0x008
#define DSPL_F			0x004

#define DSPL_CTRL		0x108
#define DSPL_D			0x004
#define DSPL_C			0x002
#define DSPL_B			0x001

#define DSPL_ENTRYMODE		0x104
#define DSPL_ID			0x002
#define DSPL_S			0x001

#define DSPL_CURSORSHIFT	0x110
#define DSPL_SC			0x008
#define DSPL_RL			0x004


// Shift LED

#define SHFTLED			0x0001


// Incremental Encoder

#define INCENC_INCSHFT		0x0002
#define INCENC_SHFTINC_MASK	0x003c
#define INCENC_AUTOSHFT		0x0040
#define INCENC_AUTORES		0x0100

#define INCENC_UF		0x08000000
#define INCENC_OV               0x10000000


/********************************************************************************
 *  Macros
 ********************************************************************************/

#define to_digit(x) ((x) + '0')


/********************************************************************************
 *  Types
 ********************************************************************************/

enum keyT {
	KEY_F1,
	KEY_F2,
	KEY_SHFT,
	KEY_F3,
	KEY_F4,
	KEY_ESC,
	KEY_ENTER
};

enum edgeT {
	rising_edge,
	falling_edge
};


/********************************************************************************
 *  Functions
 ********************************************************************************/

// - Display -

void display_init(bool cursor, bool blink, bool increment, bool shift);
void display_clear();
void display_putstring(char* text);
void display_putchar(char c);
void display_move(bool right);
void display_cursor(bool visible, bool blink);
void display_generate_char(unsigned code, char sign[8]);

void display_gotohome();

void display_gotoxy(unsigned x, unsigned y);
bool display_full();
void display_putuint(unsigned u);
void display_putint_dec(int i);


// - Keys -

bool key_pressed(enum keyT key);
void key_reset_trig();
bool key_trig(enum edgeT edge, enum keyT key);


// - Incremental Encoder -

unsigned incenc_read(int *pos);
void incenc_write(int i);
void incenc_set_shftinc(unsigned u);
void incenc_set_autores(bool on);
void incenc_set_autoshft(bool on);
void shftled_set(bool on);

#endif // COMUNIT_H
