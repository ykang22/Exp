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
signed extract_signed(const unsigned input){
	return (signed)(input<<(32u - end)) >> (32u - end + begin);
}


#if defined VDSP && !defined DEBUG && defined __2116x__

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(suppress:609)
#endif

#pragma const
template<> inline signed extract_signed<0,32>(const unsigned i){ return i; }
#pragma const
template<> inline signed extract_signed<0,1>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,2>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,3>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,4>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,5>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:28 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:29 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:30 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<0,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 0:31 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,2>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,3>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,4>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,5>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:28 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:29 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:30 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<1,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 1:31 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,3>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,4>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,5>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:28 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:29 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<2,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 2:30 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,4>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,5>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:28 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<3,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 3:29 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,5>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<4,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 4:28 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,6>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<5,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 5:27 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,7>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<6,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 6:26 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,8>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<7,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 7:25 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,9>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<8,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 8:24 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,10>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<9,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 9:23 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,11>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<10,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 10:22 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,12>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<11,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 11:21 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,13>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<12,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 12:20 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,14>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<13,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 13:19 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,15>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<14,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 14:18 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,16>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<15,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 15:17 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,17>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<16,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 16:16 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,18>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<17,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 17:15 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,19>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<18,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 18:14 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,20>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<19,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 19:13 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,21>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<20,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 20:12 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,22>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<21,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 21:11 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,23>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<22,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 22:10 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,24>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<23,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 23:9 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,25>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<24,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 24:8 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,26>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<25,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 25:7 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,27>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<26,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 26:6 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<27,28>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 27:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<27,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 27:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<27,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 27:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<27,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 27:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<27,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 27:5 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<28,29>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 28:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<28,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 28:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<28,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 28:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<28,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 28:4 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<29,30>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 29:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<29,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 29:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<29,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 29:3 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<30,31>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 30:1 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<30,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 30:2 (SE);" : "=D"(o) : "D"(i)); return o; }
#pragma const
template<> inline signed extract_signed<31,32>(const unsigned i){ signed o; asm("%0 = FEXT %1 BY 31:1 (SE);" : "=D"(o) : "D"(i)); return o; }

#if defined __VERSIONNUM__ && __VERSIONNUM__ > 0x07000000
	#pragma diag(warning:609)
#endif

#endif // VDSP
