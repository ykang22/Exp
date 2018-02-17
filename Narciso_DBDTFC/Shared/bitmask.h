/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

template<const unsigned begin, const unsigned end>
class bitmask{
public:
	operator unsigned(void)const{
		return ((1u<<(end-begin))-1)<<begin;
	}
};

template<>
class bitmask<0,32>{
public:
	operator unsigned(void)const{
		return 0xFFFFFFFF;
	}
};

#define BIT00 0x00000001u
#define BIT01 0x00000002u
#define BIT02 0x00000004u
#define BIT03 0x00000008u
#define BIT04 0x00000010u
#define BIT05 0x00000020u
#define BIT06 0x00000040u
#define BIT07 0x00000080u
#define BIT08 0x00000100u
#define BIT09 0x00000200u
#define BIT10 0x00000400u
#define BIT11 0x00000800u
#define BIT12 0x00001000u
#define BIT13 0x00002000u
#define BIT14 0x00004000u
#define BIT15 0x00008000u
#define BIT16 0x00010000u
#define BIT17 0x00020000u
#define BIT18 0x00040000u
#define BIT19 0x00080000u
#define BIT20 0x00100000u
#define BIT21 0x00200000u
#define BIT22 0x00400000u
#define BIT23 0x00800000u
#define BIT24 0x01000000u
#define BIT25 0x02000000u
#define BIT26 0x04000000u
#define BIT27 0x08000000u
#define BIT28 0x10000000u
#define BIT29 0x20000000u
#define BIT30 0x40000000u
#define BIT31 0x80000000u
#define BIT0 BIT00
#define BIT1 BIT01
#define BIT2 BIT02
#define BIT3 BIT03
#define BIT4 BIT04
#define BIT5 BIT05
#define BIT6 BIT06
#define BIT7 BIT07
#define BIT8 BIT08
#define BIT9 BIT09
