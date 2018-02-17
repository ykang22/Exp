function [] = AIX_close()

calllib('XCSDataLink', 'Close' );
pause(0.4)
unloadlibrary XCSDataLink;
