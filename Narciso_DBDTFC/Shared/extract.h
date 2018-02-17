/**
 * 2005-10-26:	suppressed warning cc0609
 * 2005-11-08:	suppression of warning enabled only for higher versions of compiler
 **/

#pragma once
#include"always.h"

#ifdef VDSP
	#pragma const
#endif
template<unsigned begin,unsigned end>
inline unsigned extract(const unsigned input){
	unsigned beg = begin;
	unsigned en = end;
	unsigned a = input>>begin;
	unsigned b = 32u-end+begin;
	unsigned c = 0xFFFFFFFF>>b;
	unsigned d = a & c;
	unsigned e = (input>>begin) & (0xFFFFFFFF>>(32u-end+begin));
	return d;
//	return (input>>begin) & (0xFFFFFFFF>>(32u-end+begin));
}

#if defined VDSP && !defined DEBUG && defined __2116x__

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	//#pragma warning(disable:609) MS-Style
	#pragma diag(suppress:609) // VDSP-Style
#endif

#pragma const
template<> inline unsigned extract<0,1>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,2>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,3>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,4>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,5>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:28;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:29;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:30;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 0:31;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<0,32>(const unsigned i){ return i; }
#pragma const
template<> inline unsigned extract<1,2>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,3>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,4>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,5>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:28;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:29;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:30;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<1,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 1:31;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,3>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,4>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,5>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:28;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:29;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<2,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 2:30;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,4>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,5>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:28;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<3,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 3:29;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,5>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<4,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 4:28;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,6>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<5,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 5:27;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,7>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<6,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 6:26;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,8>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<7,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 7:25;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,9>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<8,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 8:24;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,10>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<9,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 9:23;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,11>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<10,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 10:22;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,12>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<11,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 11:21;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,13>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<12,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 12:20;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,14>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<13,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 13:19;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,15>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<14,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 14:18;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,16>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<15,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 15:17;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,17>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<16,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 16:16;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,18>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<17,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 17:15;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,19>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<18,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 18:14;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,20>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<19,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 19:13;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,21>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<20,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 20:12;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,22>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<21,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 21:11;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,23>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<22,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 22:10;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,24>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<23,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 23:9;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,25>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<24,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 24:8;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,26>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<25,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 25:7;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,27>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<26,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 26:6;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<27,28>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 27:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<27,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 27:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<27,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 27:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<27,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 27:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<27,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 27:5;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<28,29>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 28:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<28,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 28:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<28,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 28:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<28,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 28:4;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<29,30>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 29:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<29,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 29:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<29,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 29:3;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<30,31>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 30:1;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<30,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 30:2;" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline unsigned extract<31,32>(const unsigned i){ unsigned o; asm("%0 = FEXT %1 BY 31:1;" : "=D"(o) : "D"(i)); return o; }

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(warning:609)
#endif

#endif // VDSP
