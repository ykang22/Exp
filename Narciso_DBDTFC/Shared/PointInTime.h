/**
 * 2005-03-16:	created
 * 2005-03-25:	optimized update()
 * 2005-10-26:	avoided warning cc1640
 **/

#include"clock.h"
#include"trunc.h"

struct TimeStamp{
	unsigned ms;
	unsigned ls;
};

class PointInTime{
	TimeStamp ts;
public:
	/**
	 * Constructs a new PointInTime from EMUCLK:
	 **/
	PointInTime(){
		update();
	}
	
	/**
	 * Bring TimeStamp up-to-date from EMUCLK.
	 **/
	// 8 cycles
	inline void update(){
		// Read both EMUCLK-registers twice.
		// In case that an overflow occured between reading the MSW and the LSW, the second reading is taken.
		// Results are invalid, only if the DSP hangs for 53 seconds between the second reading of MSW and LSW.
		unsigned ms1,ls1,ms2,ls2;
		asm("%0=EMUCLK2;" : "=d"(ms1));
		asm("%0=EMUCLK;"  : "=d"(ls1));
		asm("%0=EMUCLK2;" : "=d"(ms2));
		asm("%0=EMUCLK;"  : "=d"(ls2));
		asm("comp(%1,%2); if eq %0=%3;" : "+d"(ls2) : "d"(ms1), "d"(ms2), "d"(ls1));
		ts.ms = ms2;
		ts.ls = ls2;
		/*
		unsigned ms1,ls1,ms2,ls2;
		asm("%0=EMUCLK2; %1=EMUCLK; %2=EMUCLK2; %3=EMUCLK;" : "=d"(ms1), "=d"(ls1), "=d"(ms2), "=d"(ls2) );
		if(ms1 == ms2){
			ts.ms = ms1;
			ts.ls = ls1;
		}else{
			ts.ms = ms2;
			ts.ls = ls2;
		}
		*/
	}

	/**
	 * Copy-Constructor
	 **/
	PointInTime(const PointInTime & z)
	: ts(z.ts)
	{};

	bool operator<(const PointInTime & z){
		return (ts.ms < z.ts.ms) | ((ts.ms == z.ts.ms) & (ts.ls < z.ts.ls));
	};

	PointInTime & operator+=(const float delta_){
		float delta = delta_;

		const unsigned delta_ms = trunc(delta * dsp_clock() * (1.f/4294967296.f));

		delta -= delta_ms * dsp_cycle_time() * 4294967296.f;

		const unsigned delta_ls = trunc(delta * dsp_clock());
		ts.ms += delta_ms;
		const unsigned ovfl = (ts.ls >= (ts.ls += delta_ls));
		ts.ms += ovfl;
		
		return *this;
	};

	operator const TimeStamp&()const{ return ts; };
	operator TimeStamp&(){ return ts; };
};

/**
 * Returns the difference between to TimeStamps in seconds.
 **/
#pragma const
static inline float operator-(const TimeStamp &t1, const TimeStamp &t0){
	return (4294967296.f * (t1.ms-t0.ms) + (t1.ls-t0.ls)) * dsp_cycle_time();
}

