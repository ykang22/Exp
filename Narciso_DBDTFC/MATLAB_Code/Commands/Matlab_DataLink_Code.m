%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%   GTS_GMR_MERGE Matlab Control Code Examples
%%%   
%%%   This program is to act as an example of the pieces of code that can
%%%   be used to send commands and gather data from the AIX running code
%%%   compiled from project GTS_GMR_MERGE. The separate pieces of this code
%%%   can be added to scripts with loops or user input however you want.
%%%
%%%   Originally written by Tyler Brauhn, 2014-09-24
%%%   r1.0 - tested and released 2014-09-24,TB
%%%   r2.0 - add lines to control and record active decpling 2014-09-29,TB
%%%   r3.0 - made updates to reflect new method of doing offsets and gains
%%%          in AIX code, method of adding multiple decoupling cases

%%%   Rewritten by Narciso Genovese Marmolejo to suit your mood, 'cuz
%%%   oh you're so smooth, 03-07-2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% SET UP MATLAB AND OPEN DATALINK

% This clears all existing variables, close all plots, and clears the 
% workspace. Only use this if you do not want to carry over anything that 
% you were previously doing in Matlab.
    clear all;
    close all;
    clc;

% This below gets Matlab ready to communicate with the AIX. You must have
% a folder named "Commands" in the same directory as your m-file. This
% folder must have the following files:
% AIX_close.m, AIX_cmds.m, AIX_cmds.mat, AIX_get_data.m,
% AIX_get_gain_data.m, AIX_get_offset_data.m, AIX_get_sync_data.m,
% AIX_open_datalink.m, AIX_write_cmd.m, AIX_write_cmd_read.m
    addpath('Commands')
    AIX_open_datalink();
    AIX_cmds();
    fprintf('loading commands...');
    load AIX_cmds;
    fprintf('success\r');
%     analog_out_selection = 1
%     AIX_write_cmd(CMD_DISPLAY,analog_out_selection)
%{    
% Changing the variable "analog_out_selection" below changes which
% variables are output on the analog outputs 4, 5, 6, and 7.
% The analog output is limited to +/-10V, so if the output reaches these
% limits, the variable may need to be scaled in the AIX code.
% The AIX program is currently set for the following choices:
% analog_out_selection = 0: i_a_1, i_b_1, i_c_1, i_a_2
% analog_out_selection = 1: i_a_2, i_b_2, i_c_2, i_a_1
% analog_out_selection = 2: GMR_A_OUT, GMR_B_OUT, GMR_C_OUT, i_a_2
% analog_out_selection = 3: GMRfan_average, fan_frequency, fan_time, GMR_A_OUT_fan_removed
% analog_out_selection = 4: ANtest5, GMR_C_OUT_offset_removed_by_temperature, GMR_C_OUT, i_c_2
% analog_out_selection = 5: GMR_A_E2, GMR_B_E2, GMR_C_E2, i_a_2
% analog_out_selection = 6: theta_1, theta_2, w_bar_1, w_bar_2
% analog_out_selection = 7: i_a_2_GMR, i_a_2, i_b_2_GMR, i_b_2
% analog_out_selection = 8: i_a_2_GMR, i_a_2, i_c_2_GMR, i_c_2
% analog_out_selection = 9: executionTime, rand_num, LSYNC0, timeTest
% analog_out_selection = 10: i_dqs_s_2.q/10.0f, i_dqs_s_2.d/10.0f, i_dqs_e_2.q/10.0f, i_dqs_e_2.d/10.0f
%     analog_out_selection = 1;
%     fprintf('setting analog outputs to%2.0f...',analog_out_selection)
%     AIX_write_cmd(CMD_DISPLAY,analog_out_selection);
%     fprintf('success\r');
%     pause(2);
%}
%% SET UP INITIAL CONDITIONS

% PRE-LOAD TORQUE
% t_preload = 1; % Field-oriented control
% t_preload = 2; % Dead-beat direct torque and flux control
% t_preload = 3; % Volts-Herz control
% 
%     t_preload_2 = 1;
%     t_preload_1 = 1;


%% COMMANDS

% STOP COMMAND
% This puts the drive into the "STOP" state. The speed qickly goes to zero 
% and the brake resistor turns on. Don't leave the drive in the drive in
% this state too long as the brake resistor will continue to absorb power
% and will get hot.
    fprintf('stopping drive...');
    AIX_write_cmd(CMD_STOP,1);
    fprintf('success\r');

%     
%     AIX_write_cmd(CMD_PWM_FREQUENCY_2,20000);
    
% PRELOAD COMMAND
% This command sets the pre-load using the variable "t_preload". Start low 
% and increase as needed from there. Higher pre-load = higher current.
% Attempting to start from zero speed at a very high pre-load may blow the
% breaker, but increasing to the same pre-load after running may be ok.

 %{
% SPEED COMMAND FOR MACHINE 2
% The AIX program allows the speed command to be given as a constant value
% or as a sine wave.
% To give the speed command as a constant, only one value is needed.
	omega_star_constant =0;
% To give the speed command as a sine, three values are needed. This method
% can still be used to give a constant speed by setting amplitude to zero
% and offset to non-zero. Do not set the frequency to zero. A very fast
% frequency may require acceleration that pulls too much current.
    omega_star_offset = 2*pi;
    omega_star_amp = 5;
    omega_star_freq = 1;
    
% SPEED COMMAND TYPE SELECT FOR MACHINE 2
% This command selects either a constant or sinusoidal speed command.
% Using the sinusoidal speed command is preferred even when giving a
% constant speed command due to the recording commands found below.
    speed_command_select = 2; %Reset(0) Constant(1)  Sine(2) US06(3)
    fprintf('setting speed command type to %2.0f...',speed_command_select);
    AIX_write_cmd(CMD_OMEGA_SELECT_2,speed_command_select);
    fprintf('success\r');
    
% CONSTANT SPEED COMMAND (use either this or SINE SPEED COMMAND)
% This command sets the speed command using variable "omega_star_constant".
% The speed command type select must be set to 1.
    fprintf('setting constant speed command to %2.1f rad/s...',omega_star_constant);
    AIX_write_cmd(CMD_OMEGA_STAR_2,0);
    fprintf('success\r');    
%}
    
% READY COMMAND
% PUT DRIVE INTO READY BEFORE TURNING ON 480vac
% This puts the drive into the "READY" state. The gate signals will begin
% switching at 50%, i.e. the switching will begin but the voltage will stay
% at zero.
    fprintf('entering ready...');
    AIX_write_cmd(CMD_READY,1);
    fprintf('success\r');
    
% STARTUP COMMAND
% This runs the drive through the "STARTUP" state as specified in the AIX
% program. The drive will then automatically enter the "RUN" state. At this
% point the drive will respond to commands to speed and pre-load. To stop
% the drive slowly, a speed command of zero may be given. To stop the drive
% quickly, a stop command may be given.
    fprintf('starting drive...');
    AIX_write_cmd(CMD_START,1);
    fprintf('success\r');
%     prompt = '\nPress 1 to continue or 0 to exit\n';
%     flag_disconnect = input(prompt);

    t_preload_2 = 1;
    t_preload_1 = 1;
    fprintf('preloading...');
    AIX_write_cmd(CMD_TORQUE_MOD_SELECT_2,t_preload_2);
    AIX_write_cmd(CMD_TORQUE_MOD_SELECT_1,t_preload_1);
    AIX_write_cmd(CMD_T_PRELOAD,0);
    fprintf('%2.1f Nm...',t_preload_2);
    fprintf('success\r');

    
%% SPEED COMMAND FOR MACHINE 1

% SPEED COMMAND FOR MACHINE 1
% Choose the desired speed for machine 1 directly (assumes constant speed).
%     speed_command_select = 1; %Reset(0) Constant(1)  Sine(2) US06(3)
% %     fprintf('setting speed command type to %2.0f...',speed_command_select);
%     AIX_write_cmd(CMD_OMEGA_SELECT_2,speed_command_select);
% %     fprintf('success\r');
%     speed_command = 10;
%     fprintf('setting speed command type to %2.0f...',...
%         speed_command);
%     AIX_write_cmd(CMD_OMEGA_STAR_2,speed_command);
%     fprintf('success\r');

% % TORQUE COMMAND FOR MACHINE 1
 CMD_FOC_TORQUE_STAR_2=317;
    torque_command = 0.035;
% %     fprintf('setting torque command type to %2.0f...',...
% %         speed_command);
    AIX_write_cmd(CMD_FOC_TORQUE_STAR_2,torque_command);
%     AIX_write_cmd(CMD_FOC_IQS_SINE_WAVE_,torque_command);
    fprintf('success\r');
    %{
% TORQUE COMMAND FOR MACHINE 2
%     fprintf('commanding torque to machine 2...\n');
%     AIX_write_cmd(CMD_FOC_IQS_E_STAR_2,0);
%     fprintf('success\r');
     
% CHIRP DIRECTION COMMAND FOR MACHINE 2 (-1 for neg +1 for pos)
%     AIX_write_cmd(CMD_CHIRP_DIRECTION,1);
    
% % COMMAND PWM DIVISOR
%     AIX_write_cmd(CMD_PWM_FREQUENCY_2,1);
    
% % TORQUE COMMAND (SQUARE) FOR MACHINE 2    
%     fprintf('commanding square wave torque to machine 2...\n');
%     AIX_write_cmd(CMD_DB_TORQUE_STAR_SQURE_2,0);
%     fprintf('success\r');
    

%     AIX_write_cmd(CMD_CURRENT_OBS_LOW_2,0);

% SINE IQS CURRENT FOR MACHINE 2
%     fprintf('commanding sinusoidal sine wave to machine 2...\n');
%     AIX_write_cmd(CMD_FOC_IQS_SINE_WAVE_2,0);
% %     AIX_write_cmd(CMD_FOC_IQS_SINE_WAVE_FREQ_2, 0);
% %     AIX_write_cmd(CMD_FOC_IQS_SINE_WAVE_AMP_2, 0);
%     fprintf('success\r');

% % NOISE INJECTION
%     AIX_write_cmd(CMD_NOISE,0);

    %}
%{   
%% SPEED COMMAND FOR MACHINE 2


% SPEED COMMAND FOR MACHINE 2
% The AIX program allows the speed command to be given as a constant value
% or as a sine wave.
% To give the speed command as a constant, only one value is needed.
	omega_star_constant =0;
% To give the speed command as a sine, three values are needed. This method
% can still be used to give a constant speed by setting amplitude to zero
% and offset to non-zero. Do not set the frequency to zero. A very fast
% frequency may require acceleration that pulls too much current.
    omega_star_offset = 100;
    omega_star_amp = 75;
    omega_star_freq = 1;
    
% SPEED COMMAND TYPE SELECT FOR MACHINE 2
% This command selects either a constant or sinusoidal speed command.
% Using the sinusoidal speed command is preferred even when giving a
% constant speed command due to the recording commands found below.
    speed_command_select =1; %Reset(0) Constant(1)  Sine(2) US06(3)
    fprintf('setting speed command type to %2.0f...',speed_command_select);
    AIX_write_cmd(CMD_OMEGA_SELECT_2,speed_command_select);
    fprintf('success\r');
    
% CONSTANT SPEED COMMAND (use either this or SINE SPEED COMMAND)
% This command sets the speed command using variable "omega_star_constant".
% The speed command type select must be set to 1.
    fprintf('setting constant speed command to %2.1f rad/s...',omega_star_constant);
    AIX_write_cmd(CMD_OMEGA_STAR_2,0);%omega_star_constant);
    fprintf('success\r');
    
% % SINE SPEED COMMAND (use either this or CONSTANT SPEED COMMAND)
% % This command sets the speed command using variable "omega_star_offset",
% % "omega_star_amp", and "omega_star_freq".
% % The speed command type select must be set to 2.
%     fprintf('setting constant speed command to %2.1f rad/s offset, ',omega_star_offset);
%     fprintf(' %2.1f rad/s amp, ',omega_star_amp);
%     fprintf(' %2.1f Hz...',omega_star_freq);
%     AIX_write_cmd(CMD_OMEGA_SINE_OFFSET_2,omega_star_offset);
%     AIX_write_cmd(CMD_OMEGA_SINE_AMP_2,omega_star_amp);    
%     AIX_write_cmd(CMD_OMEGA_SINE_FREQ_2,omega_star_freq);
%     fprintf('success\r');
%}

%{ 
%% GMR COMMANDS

% SET GMR GAIN
% These commands set the initial gain that is applied to the measured
% signal from the GMRs. This gain does not include gains added later as
% part of the decoupling switch case found in the AIX code.
% It is recommended to leave the GAINS AT 1 and deal with necessary gains
% to get current signals in the decoupling area.
    GMRgain(1)=1; %GMRgain(1)=2.2995; %A/V
    GMRgain(2)=1; %GMRgain(2)=-3.8247; %A/V
    GMRgain(3)=1; %GMRgain(3)=-2.1275; %A/V
    GMRgain(4)=1;
    GMRgain(5)=1;
    GMRgain(6)=1;
    fprintf('setting GMR gains...');
    AIX_write_cmd(CMD_SET_GMR_A_OUT_GAIN,GMRgain(1));
    AIX_write_cmd(CMD_SET_GMR_B_OUT_GAIN,GMRgain(2));
    AIX_write_cmd(CMD_SET_GMR_C_OUT_GAIN,GMRgain(3));
    AIX_write_cmd(CMD_SET_GMR_A_E2_GAIN,GMRgain(4));
    AIX_write_cmd(CMD_SET_GMR_B_E2_GAIN,GMRgain(5));
    AIX_write_cmd(CMD_SET_GMR_C_E2_GAIN,GMRgain(6));
    fprintf('success\r');
    
% SET GMR OFFSET
% These commands set the initial offset that is applied to the measured
% signal from the GMRs. This gain does not include gains added later as
% part of the decoupling switch case or automatically determined by 
% comparing them against the LEM sensors as found in the AIX code.
% It is recommended to leave the OFFSET AT 0 and deal with necessary
% offsets to get current signals in the decoupling area or by applying the
% comparison against LEM.
    GMRoffset(1)=0; 
    GMRoffset(2)=0; 
    GMRoffset(3)=0; 
    GMRoffset(4)=0;
    GMRoffset(5)=0;
    GMRoffset(6)=0;
    fprintf('setting GMR offsets...');
    AIX_write_cmd(CMD_SET_GMR_A_OUT_OFF,GMRoffset(1));
    AIX_write_cmd(CMD_SET_GMR_B_OUT_OFF,GMRoffset(2));
    AIX_write_cmd(CMD_SET_GMR_C_OUT_OFF,GMRoffset(3));
    AIX_write_cmd(CMD_SET_GMR_A_E2_OFF,GMRoffset(4));
    AIX_write_cmd(CMD_SET_GMR_B_E2_OFF,GMRoffset(5));
    AIX_write_cmd(CMD_SET_GMR_C_E2_OFF,GMRoffset(6));
    fprintf('success\r');
    
% AUTO ZERO OFFSET -- ONLY USE DURING *READY*
% DISABLED TO SAVE MEMORY IN AIX
% This command compares the current as measured by the GMR and the current
% as measured by the LEM sensors in order to adjust the offset of the GMR
% output in order to compensate for temperature effects on the biasing
% magnet. If the temperature can be sensed and correlated to the offset,
% then this step should no longer be used, as it requires the LEM sensors.
%     fprintf('setting GMR offset...');
%     AIX_write_cmd(CMD_SET_AUTO_ALL_ZERO,1);
%     fprintf('success\r');
    
% AUTO ADJUST OFFSET -- ONLY USE DURING *RUN*
% This command compares the current as measured by the GMR and the current
% as measured by the LEM sensors in order to adjust the offset of the GMR
% output in order to compensate for temperature effects on the biasing
% magnet. If the temperature can be sensed and correlated to the offset,
% then this step should no longer be used, as it requires the LEM sensors.
    fprintf('setting GMR offset...');
    AIX_write_cmd(CMD_SET_AUTO_ALL_ADJUST,1);
    fprintf('success\r');
    
% ACTIVE DECOUPLING OFF
% The AIX program starts using the current measurements from the E2 GMR in
% order to decouple the effect of the E2 lead and other disturbances from
% outside of the system
    fprintf('turning off active decoupling...')
    AIX_write_cmd(CMD_ACTIVE_DECOUPLING_OFF,1);
    fprintf('success\r');
    
% ACTIVE DECOUPLING SELECTION
% The AIX program starts using the current measurements from the GMRs
% decoupled via any one of the following methods
% active_decoupling_selection = 0; Simple gains on C1E2 GMR equals current
% active_decoupling_selection = 1; Use E2 to decouple fan noise
% active_decoupling_selection = 2; Temperature driven offset???
% active_decoupling_selection = 3; Temperature driven offset without fan decoupling - simple gains
% active_decoupling_selection = 4; Bias magnet decoupling???
% active_decoupling_selection = 5; Fan from extra GMR???
    active_decoupling_selection = 4;
    fprintf('turning on active decoupling using second GMR...')
    AIX_write_cmd(CMD_ACTIVE_DECOUPLING_ON,active_decoupling_selection);
    fprintf('success\r');
    
% CLOSE LOOP WITH GMR
% The AIX program starts using the current measurements from the LEM for 
% closed-loop current control, and this command switches to using the GMR 
% current measurements for the closed-loop control
    fprintf('closing loop with GMR-based current sensing...')
    AIX_write_cmd(CMD_SET_GMR_CONT_ON,1);
    fprintf('success\r');
    
% CLOSE LOOP WITH LEM
% After using the command above to use GMRs for current sensing, this
% command will go back to using the LEM sensors for current sensing.
    fprintf('closing loop with LEM-based current sensing...')
    AIX_write_cmd(CMD_SET_GMR_CONT_OFF,1);
    fprintf('success\r');
 %}  
%% DATA RECORDING COMMANDS
    %% UNNECESSARY
%{
% GET PRE-SET GMR OFFSET DATA
% This returns the offset variables that are preset either in the header 
% file or by sending commands through datalink.
% The "offset_variable..." variables should be changed to an array by
% adding (i) or something when recording data in a loop, etc.
    fprintf('getting GMR pre-set offsets...')
    GMRoffset =AIX_get_offset_data();
    preset_offset_GMR_A_OUT = GMRoffset(1);
    preset_offset_GMR_B_OUT = GMRoffset(2);
    preset_offset_GMR_C_OUT = GMRoffset(3);
    preset_offset_GMR_A_E2 = GMRoffset(4);
    preset_offset_GMR_B_E2 = GMRoffset(5);
    preset_offset_GMR_C_E2 = GMRoffset(6);
    fprintf('success\r')
    
% GET PRE-SET GMR GAIN DATA
% This returns the offset variables that are preset either in the header 
% file or by sending commands through datalink.
% The "offset_variable..." variables should be changed to an array by
% adding (i) or something when recording data in a loop, etc.
    fprintf('getting GMR pre-set gains...')
    GMRgain =AIX_get_gain_data();
    preset_gain_GMR_A_OUT = GMRgain(1);
    preset_gain_GMR_B_OUT = GMRgain(2);
    preset_gain_GMR_C_OUT = GMRgain(3);
    preset_gain_GMR_A_E2 = GMRgain(4);
    preset_gain_GMR_B_E2 = GMRgain(5);
    preset_gain_GMR_C_E2 = GMRgain(6);
    fprintf('success\r')
    
% GET GMR LEM SET OFFSET DATA
% Since the AIX can set the offset of the GMR measurements automatically,
% it is not possible to tell what each was set to. This command will obtain
% the values from the AIX and record them to the "offset_variable..."
% variables. 
% The "offset_variable..." variables should be changed to an array by
% adding (i) or something when recording data in a loop, etc.
    fprintf('getting GMR offsets from LEM...')
    GMRoffsetLEM =AIX_get_offset_from_lem_data();
    offset_from_LEM_GMR_A_OUT = GMRoffsetLEM(1);
    offset_from_LEM_GMR_B_OUT = GMRoffsetLEM(2);
    offset_from_LEM_GMR_C_OUT = GMRoffsetLEM(3);
    offset_from_LEM_GMR_A_E2 = GMRoffsetLEM(4);
    offset_from_LEM_GMR_B_E2 = GMRoffsetLEM(5);
    offset_from_LEM_GMR_C_E2 = GMRoffsetLEM(6);
    fprintf('success\r')
    
% GET PCB GMR OFFSET DATA
% This returns the PCB offset variables that are preset either in the 
% header file.
% The "offset_variable..." variables should be changed to an array by
% adding (i) or something when recording data in a loop, etc.
%     fprintf('getting GMR PCB offsets...')
%     GMRoffsetPCB =AIX_get_PCB_offset_data();
%     PCB_offset_GMR_A_OUT = GMRoffsetPCB(1);
%     PCB_offset_GMR_B_OUT = GMRoffsetPCB(2);
%     PCB_offset_GMR_C_OUT = GMRoffsetPCB(3);
%     PCB_offset_GMR_A_E2 = GMRoffsetPCB(4);
%     PCB_offset_GMR_B_E2 = GMRoffsetPCB(5);
%     PCB_offset_GMR_C_E2 = GMRoffsetPCB(6);
%     fprintf('success\r')
    
% GET PCB GMR GAIN DATA
% This returns the PCB offset variables that are preset either in the 
% header file.
% The "offset_variable..." variables should be changed to an array by
% adding (i) or something when recording data in a loop, etc.
%     fprintf('getting GMR PCB gains...')
%     GMRgainPCB =AIX_get_PCB_gain_data();
%     PCB_gain_GMR_A_OUT = GMRgainPCB(1);
%     PCB_gain_GMR_B_OUT = GMRgainPCB(2);
%     PCB_gain_GMR_C_OUT = GMRgainPCB(3);
%     PCB_gain_GMR_A_E2 = GMRgainPCB(4);
%     PCB_gain_GMR_B_E2 = GMRgainPCB(5);
%     PCB_gain_GMR_C_E2 = GMRgainPCB(6);
%     fprintf('success\r')
%}
    
%% THE GOOD STUFF   
% % GET DATASET 1 *SEE NOTE BELOW
% % This dataset includes LEM measured current, speed, and torque.
% % The "dataStructCurrentSpdTq" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % % commands to the AIX is different.
    fprintf('Dataset 1...');
    AIX_write_cmd(CMD_DATASET,1);
    fprintf('clearing data set...');
    AIX_write_cmd(CMD_REC_FLAG,3);
        
    AIX_write_cmd(CMD_REC_FLAG,1);
    fprintf('recording...');
    AIX_write_cmd(CMD_NOP,1);
%     AIX_write_cmd(CMD_OMEGA_STAR_2,0);%omega_star_constant);
%     AIX_write_cmd(CMD_OMEGA_STAR_1,-100);
    
    pause(1)
    data1 = AIX_get_data();
    fprintf('got data...transferring to structure...');
%         dataStructCurrentSpdTq = struct('Test01',data1(:,1),'Test02',data1(:,2),'Test03',data1(:,3),'Test04',data1(:,4),'Test05',data1(:,5),...
%             'Test06',data1(:,6),'Test07',data1(:,7),'Test08',data1(:,8),'Test09',data1(:,9),'Test10',data1(:,10),'Dataset',1);
                dataStructCurrentSpdTq = struct('Test01',data1(:,1),'Test02',data1(:,2),'Test03',data1(:,3),'Test04',data1(:,4),'Test05',data1(:,5),'Test06',...
                    data1(:,6),'Test07',data1(:,7),'Test08',data1(:,8),'Test09',data1(:,9),'Test10',data1(:,10),'Test11',data1(:,11),'Test12',data1(:,12),'Dataset',1);
    fprintf('success\r');
    flag_disconnect = 0;
% %      pause(2);
% %      AIX_write_cmd(CMD_OMEGA_STAR_1,0);

    
    %% MORE THINGS I DON'T NEED
    %{
% % GET DATASET 5 *SEE NOTE BELOW
% % This dataset includes LEM measured current and GMR signals after PCB gain and offset.
% % The "dataStructGMRCurrent" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % commands to the AIX is different. 
%     fprintf('Dataset 5...');
%     AIX_write_cmd(CMD_DATASET,5);
%     fprintf('recording...');
%     AIX_write_cmd(CMD_REC_FLAG,3);
%     AIX_write_cmd(CMD_NOP,1);
%     pause(5)
%     data2 = AIX_get_data();
%     fprintf('got data...transferring to structure...');
%          dataStructGMRCurrent = struct('Ia2',data2(:,1),'Ib2',data2(:,2),'Ic2',data2(:,3),'GMR_A_OUT',data2(:,4),...
%         'GMR_B_OUT',data2(:,5),'GMR_C_OUT',data2(:,6),'GMR_A_E2',data2(:,7),'GMR_B_E2',data2(:,8),'GMR_C_E2',data2(:,9),'AIN5',data2(:,10),...
%         'SpeedOffset',omega_star_offset,'Freq',omega_star_freq,'SpeedAmp',omega_star_amp,...
%         'Preload',t_preload,'Dataset',5);
%     fprintf('success\r');
%     
% % GET DATASET 6 *SEE NOTE BELOW
% % This dataset includes LEM measured current and GMR signals BEFORE PCB gain and offset.
% % The "dataStructGMRCurrent" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % commands to the AIX is different. 
%     fprintf('Dataset 6...');
%     AIX_write_cmd(CMD_DATASET,6);
%     fprintf('recording...');
%     AIX_write_cmd(CMD_REC_FLAG,3);
%     AIX_write_cmd(CMD_NOP,1);
%     pause(5)
%     data2 = AIX_get_data();
%     fprintf('got data...transferring to structure...');
%          dataStructGMRRAWCurrent = struct('Ia2',data2(:,1),'Ib2',data2(:,2),'Ic2',data2(:,3),'GMR_A_OUT_RAW',data2(:,4),...
%         'GMR_B_OUT_RAW',data2(:,5),'GMR_C_OUT_RAW',data2(:,6),'GMR_A_E2_RAW',data2(:,7),'GMR_B_E2_RAW',data2(:,8),'GMR_C_E2_RAW',data2(:,9),'AIN5_RAW',data2(:,10),...
%         'SpeedOffset',omega_star_offset,'Freq',omega_star_freq,'SpeedAmp',omega_star_amp,...
%         'Preload',t_preload,'Dataset',5);
%     fprintf('success\r');
%     
% % GET DATASET 7 *SEE NOTE BELOW
% % This dataset includes LEM measured current and GMR calculated current and fan est.
% % The "dataStructGMRDecoupledCurrent" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % commands to the AIX is different. 
%     fprintf('Dataset 7...');
%     AIX_write_cmd(CMD_DATASET,7);
%     fprintf('recording...');
%     AIX_write_cmd(CMD_REC_FLAG,3);
%     AIX_write_cmd(CMD_NOP,1);
%     pause(5)
%     data3 = AIX_get_data();
%     fprintf('got data...transferring to structure...');
%          dataStructGMRDecoupledCurrent = struct('Ia2',data3(:,1),'Ib2',data3(:,2),'Ic2',data3(:,3),'Ia2_GMR',data3(:,4),...
%         'Ib2_GMR',data3(:,5),'Ic2_GMR',data3(:,6),'fan_a_GMR',data3(:,7),'fan_b_GMR',data3(:,8),'fan_c_GMR',data3(:,9),'theta2',data2(:,10),...
%         'SpeedOffset',omega_star_offset,'Freq',omega_star_freq,'SpeedAmp',omega_star_amp,...
%         'Preload',t_preload,'Dataset',7);
%     fprintf('success\r');
% 
% % GET DATASET 8 *SEE NOTE BELOW
% % This dataset includes LEM measured current and GMR calculated current and basing field est (not accurate--proportional only!).
% % The "dataStructGMRDecoupledCurrent" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % commands to the AIX is different. 
%     fprintf('Dataset 8...');
%     AIX_write_cmd(CMD_DATASET,8);
%     fprintf('recording...');
%     AIX_write_cmd(CMD_REC_FLAG,3);
%     AIX_write_cmd(CMD_NOP,1);
%     pause(5)
%     data3 = AIX_get_data();
%     fprintf('got data...transferring to structure...');
%          dataStructGMRDecoupledCurrent = struct('Ia2',data3(:,1),'Ib2',data3(:,2),'Ic2',data3(:,3),'Ia2_GMR',data3(:,4),...
%         'Ib2_GMR',data3(:,5),'Ic2_GMR',data3(:,6),'bias_a_GMR',data3(:,7),'bias_b_GMR',data3(:,8),'bias_c_GMR',data3(:,9),'GMR_C_OUT_offset_removed_by_temperature',data2(:,10),...
%         'SpeedOffset',omega_star_offset,'Freq',omega_star_freq,'SpeedAmp',omega_star_amp,...
%         'Preload',t_preload,'Dataset',8);
%     fprintf('success\r');    
%                 
% % GET DATASET 9 *SEE NOTE BELOW
% % This dataset includes LEM measured current and GMR calculated current and temperature stuff.
% % The "dataStructGMRDecoupledCurrent" variable should be changed to an array by
% % adding (i) or something when recording data in a loop, etc.
% % The last four variables for SpeedOffset, Freq, SpeedAmp, and Preload may
% % need to be changed depending on if the variable to give the speed
% % commands to the AIX is different. 
%     fprintf('Dataset 9...');
%     AIX_write_cmd(CMD_DATASET,9);
%     fprintf('recording...');
%     AIX_write_cmd(CMD_REC_FLAG,3);
%     AIX_write_cmd(CMD_NOP,1);
%     pause(5)
%     data3 = AIX_get_data();
%     fprintf('got data...transferring to structure...');
%          dataStructGMRDecoupledCurrent = struct('Ia2',data3(:,1),'Ib2',data3(:,2),'Ic2',data3(:,3),'Ia2_GMR',data3(:,4),...
%         'Ib2_GMR',data3(:,5),'Ic2_GMR',data3(:,6),'GMR_c_offset_removed_by_temp',data3(:,7),'AIN5',data3(:,8),...
%         'SpeedOffset',omega_star_offset,'Freq',omega_star_freq,'SpeedAmp',omega_star_amp,...
%         'Preload',t_preload,'Dataset',9);
%     fprintf('success\r');    
                        

    

%*NOTE!!!
% If the above commands are used to record dataset 4, 5, or 6, all of 
% the data is written to variables "data1" and/or "data2". If the command
% to record dataset 4,5, or 6 is used again, these variables will be
% OVERWRITTEN and the previously recorded data will be ERASED. To avoid
% this, either save data1 and data2 as a different variable, or make these
% variables an array with an index variable after each recording command.
       %}
%% CLOSE DATALINK

% The datalink may be left open to give commands from the Matlab command
% line; however, the datalink should be closed before resetting or powering
% off the AIX. If you don't close the datalink, Matlab will crash next time
% you try to send commands to the AIX.
% 
% if(flag_disconnect == 0)
%     AIX_close();
%     fprintf('Successfully Disconnected from AIX\n');
% end
    
 
