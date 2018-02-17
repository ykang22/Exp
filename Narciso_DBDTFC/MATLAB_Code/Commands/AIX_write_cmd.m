function [status] = AIX_write_cmd(cmd_val,data)
        
        logData = typecast(single(data),'uint32');
        cmd = [cmd_val, logData]
        status = calllib('XCSDataLink', 'Write', cmd, 8)
        pause(.4);
        fprintf('.')
