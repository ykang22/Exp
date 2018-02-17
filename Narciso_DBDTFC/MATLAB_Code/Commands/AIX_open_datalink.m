function [] = AIX_open_datalink()



loadlibrary('C:\Users\RDL\Documents\2015 From Kang_DBDTFC_Software\Code\New_XCS2000LIB&DataLinkFile\PC\XCSDataLink.dll','C:\Users\RDL\Documents\2015 From Kang_DBDTFC_Software\Code\New_XCS2000LIB&DataLinkFile\PC\XCSDataLink.h');
% loadlibrary('C:\Users\RDL\Documents\VisualDSP Projects\AIX Startup\XCSDataLink.dll','C:\Users\RDL\Documents\VisualDSP Projects\Narciso_DBDTFC\XCSDataLink.h');

% open the DataLink
status = calllib('XCSDataLink', 'Open', '192.168.252.101')
if status <0,
    msgbox('DataLink could not be opened. Please revise communication settings. Is XCS programmed, on and ready?','DataLink Setup failed', 'error');
    calllib('XCSDataLink', 'Close' );
    unloadlibrary XCSDataLink;
    return;
end;
err_string = calllib('XCSDataLink', 'GetErrorString');

