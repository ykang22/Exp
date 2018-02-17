/**
 * add_version_notes_here
 **/

#pragma once

#include"bit.h"
#include"extract_bit.h"

template<unsigned N>
class bitset{
	static inline unsigned n(){ return N/32 + (N%32)?1:0; };

public:
	unsigned m[N/32 + (N%32)?1:0];

	static bitset new_bitset(const bool b=false){
		const unsigned u = b ? 0xFFFFFFFF : 0x00000000;
		for(unsigned i=0; i<n(); ++i)
			m[i] = u;
	};

	inline bit<bitset> operator[](const unsigned pos){
		return bit<bitset>(m[pos/32], pos%32);
	};

	// Test:
	inline bool test(const unsigned pos)const{
		return dynamic_extract_bit(m[pos/32], pos%32);
	};

	// Test in any bit is set:
	inline bool test_any()const{
		unsigned u = 0;
		for(unsigned i=0; i<n(); ++i)
			u |= m[i];
		return u != 0;
	};

	inline bool operator!()const{
		return ! test_any();
	};

	// Set:
	inline bool set(const unsigned pos){
		m[pos/32] |= 1<<(pos%32);
		return true;
	};

	// Reset:
	inline bool reset(const unsigned pos){
		m[pos/32] &= ~(1<<(pos%32));
		return false;
	};

	// Toggle:
	inline bool toggle(void){
		m[pos/32] ^= 1<<(pos%32);
		return test(pos);
	};

	// Write:
	inline bitset<N>& operator=(const bitset<N>& bs){
		for(unsigned i=0; i<n(); ++i)
			m[i] = bs.m[i];
		return *this;
	};

	// Or:
	inline bitset<N>& operator|=(const bitset<N>& bs){
		for(unsigned i=0; i<n(); ++i)
			m[i] |= bs.m[i];
		return *this;
	};

	// And:
	inline bitset<N>& operator&=(const bitset<N>& bs){
		for(unsigned i=0; i<n(); ++i)
			m[i] &= bs.m[i];
		return *this;
	};

	// Xor:
	inline bitset<N>& operator^=(const bitset<N>& bs){
		for(unsigned i=0; i<n(); ++i)
			m[i] ^= bs.m[i];
		return *this;
	};
	/*
	// Rotate:
	bitset<N>& rot1right(){
		const bool b0 = test(0);
		for(unsigned i=0; i<n(); ++i){
			unsigned u=m[i];
			u >>= 1;
			u |= deposit<31,32>(m[i+1]);
			m[i] = u;
		}
		if(b0)
			set(N-1);
		else
			reset(N-1);
		return *this;
	}

	bitset<N>& rot(const signed char r){ // rotates r bits left.
		const unsigned t = (N-r)%N;
		for(unsigned i=0; i<t; ++i)
			rot1right();
		return *this;
	};
	*/
};

#define N 32u
template<>
class bitset<N>{
	static inline unsigned n(){ return N/32 + (N%32)?1:0; };

public:
	unsigned m/*[N/32 + (N%32)?1:0]*/;
	
	inline operator unsigned(void)const
	{
		return m;
	};

	inline bit<bitset> operator[](const unsigned pos)
	{
		return bit<bitset>(*this, pos%32);
	};

	inline bool operator[](const unsigned pos)const
	{
		return test(pos);
	};

	// Test:
	inline bool test(const unsigned pos)const{
		return dynamic_extract_bit(m, pos%32);
	};

	// Set:
	inline bool set(const unsigned pos){
		m |= 1<<(pos%32);
		return true;
	};

	// Reset:
	inline bool reset(const unsigned pos){
		m &= ~(1<<(pos%32));
		return false;
	};

	// Toggle:
	inline bool toggle(const unsigned pos){
		m ^= 1<<(pos%32);
		return test(pos);
	};

/*	// Write:
	inline bitset<N>& operator=(const bitset<N>& bs){
		m = bs.m;
		return *this;
	};
*/
	// Write:
	inline bitset<N>& operator=(const unsigned u){
		m = u;
		return *this;
	};

	// Or:
	inline bitset<N>& operator|=(const bitset<N>& bs){
		m |= bs.m;
		return *this;
	};

	// And:
	inline bitset<N>& operator&=(const bitset<N>& bs){
		m &= bs.m;
		return *this;
	};

	// Xor:
	inline bitset<N>& operator^=(const bitset<N>& bs){
		m ^= bs.m;
		return *this;
	};

	// Rotate:
	bitset<N>& rot(const signed char r){ // rotates r bits left.
		#ifdef VDSP
			asm("ROT %0 BY %1" : "=d"(m) : "d"(r));
		#else
			m = (m << (N-((N-r)%N))) | (m >> ((N-r)%N));
		#endif
		return *this;
	};
};

#undef N
