function [data_out] = AIX_get_data()

    load AIX_cmds
    [status,datalength]=AIX_write_cmd_read(CMD_SEND_DATA_SIZE,1);
    pause(0.4)
    fprintf('.')
    
    [status,numchan]=AIX_write_cmd_read(CMD_SEND_DATA_CHAN,1);
    pause(0.4)
    fprintf('.')

    % 5: request the data
    cmd = [CMD_SEND_DATA, 1]; 
    status = calllib('XCSDataLink', 'Write', cmd, 8);
    pause(0.4);
    % 6: read the data
    buffer(1:numchan,1:datalength) = uint32(0);
    data_ptr = libpointer('uint32Ptr',buffer);%libpointer('uint32Ptr',buffer);
    status = calllib('XCSDataLink', 'Read', data_ptr, 4*datalength*numchan);
    data_uint = data_ptr.value;
    fprintf('.')
    
    for m=1:numchan
        data_out(:,m) = typecast(data_uint(m,:),'single');
        fprintf('.');
    end   
    
    pause(0.4)