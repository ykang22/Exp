/**
 * add_version_notes_here
 **/

#pragma once

template<class T>
class bit{
	T& r;
	const unsigned pos;
	static inline unsigned setmask(const unsigned pos){ return 1u<<pos; };
	static inline unsigned resetmask(const unsigned pos){ return ~setmask(pos); };
public:
	bit(T& r_, const unsigned pos_)
	: r(r_), pos(pos_)
	{};

	bit(const bit& b)
	: r(b.r), pos(b.pos)
	{};

	inline bit& operator=(const bit& b){
		if(b.test()){
			set();
		}else{
			reset();
		}
		return *this;
	};

	// Read:
	inline operator bool()const{
		return r.test(pos);
	};
	
	// Test:
	inline bool test(void)const{
		return r.test(pos);
	};

	// Set:
	inline bit& set(void){
		r.set(pos);
		return *this;
	};

	// Reset:
	inline bit& reset(void){
		r.reset(pos);
		return *this;
	};

	// Toggle:
	inline bit& toggle(void){
		r.toggle(pos);
		return *this;
	};

	// Write:
	inline bit& operator=(const bool b){
		if(b){
			set();
		}else{
			reset();
		}
		return *this;
	};

	// Or:
	inline bit& operator|=(const bool b){
		if(b)
			set();
		return *this;
	};

	// And:
	inline bit& operator&=(const bool b){
		if(!b)
			reset();
		return *this;
	};

	// Xor:
	inline bit& operator^=(const bool b){
		if(b)
			toggle();
		return *this;
	};
};
