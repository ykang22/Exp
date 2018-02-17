#ifndef _XDT_H
#define _XDT_H

namespace DataLink {
	
	enum StatusCodes {
		XDT_OK = 0,
		XDT_ALREADY_OPEN,
		XDT_NOT_OPEN,
		
		XDT_TRANSMISSION_PENDING,
		
		XDT_INVALID_PARAMETER
	};
	
	class XCSDataLink {
	public:
		int     Open   ();
		int     Read   ( void* buffer,       unsigned length );
		int     Write  ( const void* buffer, unsigned length );
		void    Close  ();
		
		void    Update ();
	};
	
}

#endif



