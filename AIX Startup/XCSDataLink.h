#ifndef _XCSDATALINK_H
#define _XCSDATALINK_H

#ifdef __cplusplus
extern "C" {
#endif


// DLL Function signatures
int					Open			(const char* const);
int					Write			(const unsigned*, int);
int					Read			(unsigned*, int);
void				Abort			();
const char*const	GetErrorString	();
int 				Close			();

#ifdef __cplusplus
}
#endif

    
#endif