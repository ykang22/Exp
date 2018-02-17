/**
 * 2005-10-25:	removed operator-> from iterators
 **/

#pragma once
#include"always.h"
#include"bitmask.h"
#include"deposit.h"
#include"extract.h"

/**
 * packs N chars into N/4 (round up) longwords
 */
template<unsigned N>
class pstring{
public:
	unsigned d[(N+3)/4];

	char operator[](const unsigned pos)const{
		switch(pos%4){
			default:
			case 0: return extract<24,32>(d[pos/4]);
			case 1: return extract<16,24>(d[pos/4]);
			case 2: return extract<8,16>(d[pos/4]);
			case 3:	return extract<0,8>(d[pos/4]);
		};
	};

	/**
	 * Modifiable char in this pstring
	 */
	class mchar{
		unsigned &u;
		const unsigned pos;
		friend class pstring<N>;
		mchar(pstring &ps, const unsigned p) : u(ps.d[p/4]), pos(p%4){};
		mchar(unsigned &u_, const unsigned pos_) : u(u_), pos(pos_){};
	public:
		mchar& operator=(const char c){
			switch(pos){
				case 0:
					u &= ~bitmask<24,32>();
					u |= deposit<24,32>(c);
					break;
				case 1:
					u &= ~bitmask<16,24>();
					u |= deposit<16,24>(c);
					break;
				case 2:
					u &= ~bitmask<8,16>();
					u |= deposit<8,16>(c);
					break;
				case 3:
					u &= ~bitmask<0,8>();
					u |= deposit<0,8>(c);
					break;
			};
			return *this;
		};
		operator char()const{
			switch(pos){
				default:
				case 0: return extract<24,32>(u);
				case 1: return extract<16,24>(u);
				case 2: return extract<8,16>(u);
				case 3:	return extract<0,8>(u);
			};
		};
	};

	mchar operator[](const unsigned pos){
		return mchar(*this,pos);
	};

	/**
	 * char-Iterator over pstring
	 */
	class iterator{
		pstring<N>& ps;
		unsigned pos;
	public:
		iterator(pstring<N>& ps_, const unsigned pos_=0) : ps(ps_), pos(pos_) {};
		iterator(void*const ptr, const unsigned pos_=0) : ps(*reinterpret_cast<pstring<N>*>(ptr)), pos(pos_) {};
		iterator& operator++(){ ++pos; return *this; };
		iterator& operator--(){ --pos; return *this; };
		iterator& operator+=(const int i){ pos+=i; return *this; };
		iterator& operator-=(const int i){ pos-=i; return *this; };
		mchar operator*(){ return mchar(ps,pos); };
		char operator*()const{ return ps[pos]; };
		mchar operator[](const unsigned p){ return ps[p]; };
	};

	/**
	 * const char-Iterator over const pstring
	 */
	class const_iterator{
		const pstring<N>& ps;
		unsigned pos;
	public:
		const_iterator(const pstring<N>& ps_, const unsigned pos_=0) : ps(ps_), pos(pos_) {};
		const_iterator(const void*const ptr, const unsigned pos_=0) : ps(*reinterpret_cast<const pstring<N>*>(ptr)), pos(pos_) {};
		const_iterator& operator++(){ ++pos; return *this; };
		const_iterator& operator--(){ --pos; return *this; };
		const_iterator& operator+=(const int i){ pos+=i; return *this; };
		const_iterator& operator-=(const int i){ pos-=i; return *this; };
		char operator*()const{ return ps[pos]; };
		char operator[](const unsigned p)const{ return ps[p]; };
	};
};
