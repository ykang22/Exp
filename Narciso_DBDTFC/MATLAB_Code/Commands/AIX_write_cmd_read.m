function [status,data_out] = AIX_write_cmd_read(cmd_val,data_in)
        %Write out CMD
        logData = typecast(single(data_in),'uint32'); 
        cmd = [cmd_val, logData];
        status = calllib('XCSDataLink', 'Write', cmd, 8);
        pause(0.4);
        % Read back data
        buffer(1:2) = uint32(0);
        data_out_ptr = libpointer('uint32Ptr',buffer);
        status = calllib('XCSDataLink', 'Read', data_out_ptr, 8);
        data_out = data_out_ptr.value(2);
        pause(0.4);