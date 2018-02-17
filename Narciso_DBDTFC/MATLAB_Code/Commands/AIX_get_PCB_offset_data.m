function [data_out] = AIX_get_PCB_offset_data()

    load AIX_cmds
    numchan =6;
    datalength=1;
    
    % 5: request the data
    cmd = [CMD_SEND_ALL_GMR_OFFSET_PCB, 1]; 
    status = calllib('XCSDataLink', 'Write', cmd, 8);
    pause(0.05);

    % 6: read the data
    buffer(1:numchan,1:datalength) = uint32(0);
    data_ptr = libpointer('uint32Ptr',buffer);
    status = calllib('XCSDataLink', 'Read', data_ptr, 4*numchan*datalength);
    data_uint = data_ptr.value;
    for m=1:numchan
        data_out(:,m) = typecast(data_uint(m,:),'single');
        fprintf('');
    end   
    
    pause(0.05)