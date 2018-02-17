function []= AIX_cmds()
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Protocol Commands these must match header in AIX code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    CMD_NOP                         = uint32(1);
    CMD_STOP                        = uint32(10);
	CMD_START                       = uint32(11);
    CMD_READY                       = uint32(12);
    CMD_SEND_DATA                   = uint32(21);
    CMD_SEND_DATA_SIZE              = uint32(22);
    CMD_SEND_DATA_CHAN              = uint32(23);
	CMD_REC_FLAG 					= uint32(24); 
    CMD_REC_DECIMATE                = uint32(25);
    CMD_DATASET                     = uint32(26);
    %CMD_SEND_TD_DATA                = uint32(30);
    %CMD_SEND_KE_DATA                = uint32(31);
    %CMD_SEND_AVG_DATA               = uint32(32);
    %CMD_SEND_SYNC_DATA_SIZE         = uint32(33);
    %CMD_SEND_SYNC_DATA_CHAN         = uint32(34);
    CMD_BD_3                        = uint32(40);
    CMD_BD_2                        = uint32(41);
    CMD_BD_1                        = uint32(42);
    CMD_BD_0                        = uint32(43);
    CMD_KSD_3                       = uint32(44);
    CMD_KSD_2                       = uint32(45);
    CMD_KSD_1                       = uint32(46);
    CMD_KSD_0                       = uint32(47);
    CMD_KISD_3                      = uint32(48);
    CMD_KISD_2                      = uint32(49);
    CMD_KISD_1                      = uint32(50);
    CMD_KISD_0                      = uint32(51);
    CMD_BO                          = uint32(52);
    CMD_KSO                         = uint32(53);
    CMD_KISO                        = uint32(54);
    CMD_BK                          = uint32(55);
    CMD_KSK                         = uint32(56);
    CMD_KISK                        = uint32(57);    
	CMD_OMEGA_STAR_1 				= uint32(101);
    CMD_J_P                         = uint32(110);
    CMD_B_P                         = uint32(111);
    CMD_T_MU                        = uint32(112);
	CMD_OMEGA_SELECT_2				= uint32(201);
	CMD_OMEGA_STAR_2				= uint32(210);
	CMD_OMEGA_SINE_OFFSET_2			= uint32(211);
	CMD_OMEGA_SINE_AMP_2			= uint32(212);
	CMD_OMEGA_SINE_FREQ_2			= uint32(213);
    CMD_OMEGA_SLEW_2                = uint32(214);
    CMD_T_PRELOAD_STAR_AMP          = uint32(215);
    CMD_T_PRELOAD_STAR_FREQ         = uint32(216);
    CMD_T_PRELOAD_STAR_OFFSET       = uint32(217);
    
    CMD_TORQUE_MOD_SELECT_2			=uint32(240);
	CMD_TORQUE_MOD_SELECT_1			=uint32(241);
    
    CMD_VS_AMP_2                    = uint32(251);
	CMD_VS_FREQ_2                   = uint32(252);
	%CMD_VS_AMP                      = uint32(251);
	%CMD_VS_FREQ                     = uint32(252);
    CMD_FOC_IQS_E_STAR_2            = uint32(321);
	CMD_FOC_REG_P_2					= uint32(260);
	CMD_FOC_REG_I_2					= uint32(261);
    CMD_FOC_IQS_SINE_WAVE_2         = uint32(263);
    CMD_FOC_IQS_SINE_WAVE_FREQ_2    = uint32(264);
    CMD_FOC_IQS_SINE_WAVE_AMP_2     = uint32(265);
    CMD_DB_TORQUE_STAR_SQURE_2      = uint32(274);
	CMD_T_PRELOAD                   = uint32(300);
	CMD_TD_AMP                      = uint32(301);
	%CMD_AVG_LIMIT                   = uint32(302);
	CMD_TD_FREQ                     = uint32(303);  
    CMD_DISPLAY                     = uint32(310);
    CMD_FILTER                      = uint32(311);
    %CMD_FLAG_BACKLASH               = uint32(312);
    CMD_FOC_TORQUE_STAR_1           = uint32(316);
    CMD_TEST                        = uint32(400);
    CHANGE_T_COMP                   = uint32(410);   
    CHANGE_I_THOLD                  = uint32(411);
    
    CMD_SEND_ALL_GMR_OFF			= uint32(420);
    CMD_SET_GMR_A_OUT_GAIN			= uint32(421); 
    CMD_SET_GMR_A_OUT_OFF			= uint32(422); 
    
    CMD_SET_GMR_B_OUT_GAIN			= uint32(424); 
    CMD_SET_GMR_B_OUT_OFF			= uint32(425); 
    
    CMD_SET_GMR_C_OUT_GAIN			= uint32(427); 
    CMD_SET_GMR_C_OUT_OFF			= uint32(428); 
    
    CMD_SET_GMR_A_E2_GAIN			= uint32(430); 
    CMD_SET_GMR_A_E2_OFF			= uint32(431); 
    
    CMD_SET_GMR_B_E2_GAIN			= uint32(433); 
    CMD_SET_GMR_B_E2_OFF			= uint32(434); 
    
    CMD_SET_GMR_C_E2_GAIN			= uint32(436); 
    CMD_SET_GMR_C_E2_OFF			= uint32(437);
    
    CMD_SET_AUTO_ALL_ZERO            = uint32(439);
    CMD_SET_AUTO_ALL_ADJUST          = uint32(440);
    
    CMD_SET_GMR_CONT_ON              = uint32(450);
    CMD_SET_GMR_CONT_OFF              = uint32(451);
    CMD_SEND_ALL_GMR_GAIN			= uint32(452);
    
    CMD_ACTIVE_DECOUPLING_ON          = uint32(453);
    CMD_ACTIVE_DECOUPLING_OFF         = uint32(454); 
    
    CMD_SEND_ALL_GMR_OFFSET_LEM          = uint32(455);
    CMD_SEND_ALL_GMR_GAIN_PCB          = uint32(456);
    CMD_SEND_ALL_GMR_OFFSET_PCB          = uint32(457);
    
    CMD_CURRENT2_NOISE_INJECTION_OFF          = uint32(460);
    CMD_CURRENT2Q_NOISE_INJECTION_ON          = uint32(461);
    CMD_CURRENT2D_NOISE_INJECTION_ON          = uint32(462);
    
    CMD_PWM_FREQUENCY_2                      = uint32(500);
    CMD_NOISE                                = uint32(600);
    CMD_CHIRP_DIRECTION                      = uint32(601);
    CMD_CURRENT_OBS_LOW_2                 =uint32(602);
    
    save('AIX_cmds.mat');

     