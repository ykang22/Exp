/**
 * add_version_notes_here
 **/

#pragma once
#include"always.h"

#include"atomic.h"

template<class Treg,unsigned adr,class Tcache = unsigned>
class Cache{
	Treg r;

	static Tcache cache;

	inline Tcache write(const Tcache value){
		atomic_on();
		cache = value;
		r = value;
		atomic_off();
		return value;
	};
public:
	inline Tcache operator=(const Tcache value){
		return write(value);
	};

	inline operator Tcache(void)const{
		return cache;
	};

	inline Tcache operator|=(const Tcache& value){
		return write(cache | value);
	};

	inline Tcache& operator&=(const Tcache& value){
		return write(cache & value);
	};

	inline Tcache& operator^=(const Tcache& value){
		return write(cache ^ value);
	};

	inline Tcache& operator+=(const Tcache& value){
		return write(cache + value);
	};

	inline Tcache& operator-=(const Tcache& value){
		return write(cache - value);
	};

	inline Tcache& operator*=(const Tcache& value){
		return write(cache * value);
	};

	inline Tcache& operator/=(const Tcache& value){
		return write(cache / value);
	};

	inline Tcache& operator%=(const Tcache& value){
		return write(cache % value);
	};

	/* Bit-Operations */
	// Test only one bit:
	inline bool test(const unsigned pos)const{
		return ((*this)>>(pos+begin)) & 1u;
	};

	// Set only one bit:
	inline bool set(const unsigned pos){
		*this |= 1u<<pos;
		return false;
	};

	// Reset only one bit:
	inline bool reset(const unsigned pos){
		*this &= ~(1u<<pos);
		return false;
	};

	// Toggle only one bit:
	inline bool toggle(const unsigned pos){
		*this ^= 1u<<pos;
		return false;
	};

	// Read only one bit:
	inline bool operator[](const unsigned pos)const{
		return test(pos);
	};

	// Get one modifiable bit:
	inline bit<Cache<Treg,adr,Tcache> > operator[](const unsigned pos){
		return bit<Cache<Treg,adr,Tcache> >(*this,pos);
	};
};

template<class Tadr,unsigned adr,class Tcache>
Tcache Cache<Tadr,adr,Tcache>::cache = 0;
