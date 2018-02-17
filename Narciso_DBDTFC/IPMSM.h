/*************************************************************************/
/*						IPM Control Program - IPMSM.h					                   */
/*						v1.0											                                 */
/*						03/27/10											                             */
/*						Wei Xu							                                       */
/*************************************************************************/
/* v1.1  First Version   Implemented based on Jae Suk's code             */
/*        03/27/10                                                       */
/* v1.2  Integrated Selfsensing into DB-DTFC                             */
/*       Transition between HFI to BEMF Tracking   (not tested)          */
/*                                                                       */
/*************************************************************************/

#include "constants.h"

/* Definition of Constants */
static const float two_pi = 2.f * 3.14159265f;		// two PI = one revolution
static const float divroot3 = 0.57735026918963f;	// one over square root of three
  

/* Structures and defined Types */
const float pwm_frequency = 10000.f;				// switching frequency of the PWM
const float dt = 1/pwm_frequency;					// switching period


/* Analog Input macro constants */
// Channels based on ribbon cable and XCI2007 pin-out
#define IA_A	AN16      //Inverter A phase A current
#define IB_A	AN20      //Inverter A phase B current

#define IA_B	AN8       //Inverter B phase A current
#define IB_B	AN12      //Inverter B phase B current

#define VDC_A	AN17      //Inverter A DC Voltage

#define VDC_B   AN9       //Inverter B DC Voltage

/* PWM Compensation */
//------Deadtime Compensation Look-up-table------//
float DTC_LUT[41]   = {-0.03075000000000f,-0.03069625159420f,-0.03063875478261f,-0.03057813217391f,-0.03051500637681f,-0.03045000000000f,-0.03038000000000f,-0.03015000000000f,-0.02985000000000f,-0.02960000000000f,-0.02915000000000f,-0.02884679676441f,-0.02863248331648f,-0.02841977148635f,-0.02812137310415f,-0.02765000000000f,-0.02615000000000f,-0.02315000000000f,-0.01765000000000f,-0.00685000000000f,0.00235000000000f,0.00825000000000f,0.02060000000000f,0.02440000000000f,0.02705000000000f,0.02805000000000f,0.02841407649202f,0.02865247013997f,0.02882882554191f,0.02900678729589f,0.02925000000000f,0.02960000000000f,0.02995000000000f,0.03030000000000f,0.03035000000000f,0.03035000000000f,0.03036813333333f,0.03042040000000f,0.03050360000000f,0.03061453333333f,0.03075000000000f};	

/*
//Fault Flags 
#define C_FT_OC_IAS				0x1   					//Over current in A phase
#define C_FT_OC_IBS				0x2	    				//Over current in B phase
#define C_FT_OC_ICS				0x4						//Over current in C phase
#define C_FT_OV					0x8		    			//Over voltage
#define C_FT_UV					0x10			 		//Under voltage
#define C_FT_OS					0x20
#define C_FT_NEG_SQRT    		0x40					//negative squareroot (commanded flux not possible for calculated torque line
*/


/* Drive Flags */
enum {
    STARTUP,
    RUN,
    STOP,
    INIT,
    ALIGN
};	

volatile int Align_cnt = 0;

volatile int drive_flag =  STOP;	   			




/* Flags */
	int Flag_fault_A = 0;
	int Flag_fault_B = 0;
	int Flag_first_step = 0;
	int	Flag_run = 0;
	int Flag_init_pos = 0;
	int Flag_discharge = 0;
	int Flag_align = 0;
	int Flag_rec_offset = 0;
	int Flag_rec = 0;
	volatile long dump_cnt = 0;
	int Flag_source = 0;
	int Flag_ID=0;
	
//------3 phase converter variable------//
	volatile float     Uout_A[3+1]               = {0.f, 0.f, 0.f, 0.f};      // calculated output duty 
	volatile float     Uout_B[3+1]               = {0.f, 0.f, 0.f, 0.f};      // calculated output duty 	  

//-----Measured DC bus link for inverter A-----//
	float Vdc_A = 0.f;
	float Vdc_A_fil = 0.f;							//Filtered dc link voltage
	float INV_Vdc_A = 0.f;

//-----Measured DC bus link for inverter B-----//
	float Vdc_B = 0.f;
	float Vdc_B_fil = 0.f;							//Filtered dc link voltage
	float INV_Vdc_B = 0.f;
		
//-----Sensor Scale-----//
	static const float	CurrentScaleFactor	= 20.f;	// amps per volt (1:1000 sensor, 50ohm resistor, 1 turns)
		
	static const float	VoltageScaleFactor_A	= 80.72328059f;	// volts per volt (50mA:1200V, 300ohm resistor)
	static const float	VoltageScaleFactor_B	= 81.2413681f;	// volts per volt (50mA:1200V, 300ohm resistor)

	
/* Index variables */
//static int current = 0, last = 1;
		
/* Rotating High Frequency Injection */
	volatile float Wc_dT = 500*two_pi*dt;		// Carrier frequency scaled by sample time
	volatile float Vc = 30;          // Carrier signal amplitude (V)
	static float theta_c = 0;      //Carrier voltage injection angle
	float Vsd = 0;         //Sinusoidal wave injection - stationary frame - alpha axis
	float Vsq = 0;        //Sinusoidal wave injection - stationary frame - beta axis
	float Vsw = 0;        //Square wave injection
	int Count_sq = 0;     //Square wave counter
/* High Frequency Flux Pertubation*/


    float theta_lambda_hf = 0;
    float lambda_rec = 0;
    int sign = 1;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float m2 = 0.0f;
    
    float v_dmax = 0.f;
    float v_dmin = 0.f;
    float NumVsSelection = 10;
    float VsSelection = 0.f;
    float VdSelection = 0.f;
    float VqSelection = 0.f;
    float V_Circ = 0.f;
    
    
    float C_81 = 0.f;
    float C_82 = 0.f;




/********************************************************************************************************************************/
	
//-----Measured currents for inverter A-----//
	float Ias_A = 0.f, Ibs_A = 0.f, Ics_A = 0.f;
	float Ias_A_Offset = 0.f, Ibs_A_Offset = 0.f;
	float Ias_A_Offset_temp = 0.f, Ibs_A_Offset_temp = 0.f;
	
	
	float I_ds_r_A = 0.f, I_qs_r_A = 0.f;
		

//-----Measured currents for inverter B-----//
	float Ias_B = 0.f, Ibs_B = 0.f, Ics_B = 0.f;

	float Ias_B_Offset = 0.f, Ibs_B_Offset = 0.f;
	float Ias_B_Offset_temp = 0.f, Ibs_B_Offset_temp = 0.f;
	

	float I_ds_r_B = 0.f, I_qs_r_B = 0.f;
		
	int   Flag_Offset = 0, Offset_cnt = 0;
		
	
//--------Counter---------//
	float Step_Count = 0.f;
	float Wrm_Count = 0.f;
	
	float Step_Source = 0.f;

//-----Motion controller------//
	int   k_sc	= 1;
	float w_rm_com = 0.f;
	float w_rm_star = 0.f;		
	float M_i_mc_km1 = 0.f;
	float M_ii_mc_km1 = 0.f;
	float M_ii_mc_km2 = 0.f;
	float T_com_A = 0.f;
	float T_com_B = 0.f;
				
//-----Current regulator------//
	float I_ds_r_com_A = 0.f, I_qs_r_com_A = 0.f;
	float I_ds_r_com_B = 0.f, I_qs_r_com_B = 0.f;
	
	
	float v_ds_r_com_B_FF = 0.f, v_qs_r_com_B_FF = 0.f;
	float v_ds_r_com_B_ND = 0.f, v_qs_r_com_B_ND = 0.f;
	
	float v_ds_r_com_A = 0.f, v_qs_r_com_A = 0.f;
	float v_ds_r_com_B = 0.f, v_qs_r_com_B = 0.f;
	float M_i_d_km1_A = 0.f, M_i_q_km1_A = 0.f;
	float M_i_d_km1_B = 0.f, M_i_q_km1_B = 0.f;
	
	float I_ds_r_err_A = 0.f, I_qs_r_err_A = 0.f;
	float I_ds_r_err_B = 0.f, I_qs_r_err_B = 0.f;

	
	float M_p_d_A = 0.f, M_p_q_A = 0.f;
	float M_i_d_A = 0.f, M_i_q_A = 0.f;
	float M_p_d_B = 0.f, M_p_q_B = 0.f;
	float M_i_d_B = 0.f, M_i_q_B = 0.f;
	
	float I_ds_s_err_A = 0.f, I_qs_s_err_A = 0.f;
	float I_ds_s_err_B = 0.f, I_qs_s_err_B = 0.f;
	float I_ds_s_com_A = 0.f, I_qs_s_com_A = 0.f;
	float I_ds_s_com_B = 0.f, I_qs_s_com_B = 0.f;
	
	//float Coeff_LQ = 1.f;   
	
	
//-----Deadbeat direct torque controller------// Added by Jaesuk 

	// Torque commanded and measured
	float T_em_err = 0.f;		  						// T_em_com - T_em

	float m = 0.f, b = 0.f, b1 = 0.f, b2 = 0.f, b3 = 0.f, b4 = 0.f, b5 = 0.f, b6 = 0.f, b2_1 = 0.f; 
	float z_d = 0.f, z_d_1 = 0.f, z_d_2 = 0.f, z_q = 0.f, z_q_1 = 0.f, root_1_B = 0.f, root_2_B = 0.f;
	float root1 = 0.f, root2 = 0.f, root3 = 0.f;	
	float lambda_s_err = 0.f;	
	float a_d = 0.f, a_q = 0.f, r1 = 0.f, r2 = 0.f;
	float Scaling_Rs = 1.f;
	float v_ds_r_com_B_Test = 0.f, v_qs_r_com_B_Test = 0.f;
	float X_1 = 0.f, X_2 = 0.f, lam_ds = 0.f, lam_qs = 0.f; 
	float T_comB = 0.f, z_q1 = 0.f, z_q2 = 0.f;
	float b_z = 0.f;
	float lam_ds_kp1 = 0.f;
	float lam_qs_kp1 = 0.f;
	float z_d_3 = 0.f;
	float z_q_3 = 0.f;
	float lam_sl = 0.f;
	float lam_kp1_sl = 0.f;
	float lam_dqs_est = 0.f;
	float Is_r = 0.f;  //stator current in rotor reference frame
	
	float v_ds_r_ov = 0.f, v_qs_r_ov = 0.f;
	float v_ds_s_ov = 0.f, v_qs_s_ov = 0.f;
	float lambda_s_ov = 0.1f;
	int Flag_ov = 0;
	
	//loss minimizing controller (F. Quattrone)
	float l_c1 = 0.f, l_c2 = 0.f, l_k = 0.f, xn = 0.f;
	float l_f1 = 0.f, l_f2 = 0.f, l_f1_der = 0.f, l_f2_der = 0.f, l_f2_der2 = 0.f;
	float l_pc_der = 0.f, l_pc_der2 = 0.f;  //copper loss derivates 
	float l_phe_der = 0.f, l_phe_der2 = 0.f;  //hysteresis and eddy current loss derivates
	int   newtonStep = 1;
	float flux_d_opt = 0.f, flux_q_opt = 0.f;
	
	//_FOVC (F. Quattrone)
	float v_freq = 0.f;  //voltage frequency
	float fovc_ph = 0.f; //angle 
	float v_mag = 20.f;
	float dq_angle = 0.f;
	float v_freq_set = 0.f;
	float v_mag_set = 0.f;
	float dq_angle_set = 0.f;
	float v_ds_r_com_hf_B = 0.f, v_qs_r_com_hf_B = 0.f;
	
	
#ifdef _DBDTFC_B3
	float lam_ds_com = 0.f;
	float lam_qs_com = 0.f;


#endif
	
	
	
//For test
	float test_DB = 0.f;
	float test_zc = 0.f;
		
#ifdef _DBDTFC_B1	
	//Deadbeat direct torque controller------Using Torque differential Equation//
	float B_Z = 0.f;
	float M = 0.f;
	float B = 0.f;
	float lambda_s_delta = 0.f;
	float lambda_delta = 0.f;
	float X1 = 0.f;
	float X2 = 0.f;
	float zd = 0.f, zq = 0.f, zd1 = 0.f;
	float root1_B = 0.f, root2_B = 0.f;
		
#endif	
	
#ifdef _Over_Mod
	//Over Modulation
	
	float test_ov = 0.f;
	float alpha_ovm = 0.f;
	float mu = 0.f;
	
#endif

//	float V_s_mag = 0.f;
	float overmod_ratio = 1.f;
//	float V_ss_mag = 0.f;
	float V_a_peak = 0.f;
	float lambda_delta = 0.f;
	float M_P_delta = 0.f;
	float M_I_delta = 0.f;
	float overmod_ratio_last = 1.f;
	float lambda_delta_r = 0.f;
	float overmod_ratio2 = 0.f;
	float theta_v = 0.f;


//------Stationary reference frame current observer------//
	float I_ds_s_B = 0.f, I_qs_s_B = 0.f;			
	float I_ds_s_hat_B = 0.f, I_qs_s_hat_B = 0.f;
	float v_ds_s_com_B = 0.f, v_qs_s_com_B = 0.f;
	
	float I_ds_s_A = 0.f, I_qs_s_A = 0.f;
	float I_ds_s_hat_A = 0.f, I_qs_s_hat_A = 0.f;
	float v_ds_s_com_A = 0.f, v_qs_s_com_A = 0.f; 
	
		
	
//------Rotor reference frame current observer--------//
	float M_p_d_or = 0.f, M_p_q_or = 0.f;
	float M_i_d_or = 0.f, M_i_q_or = 0.f;
	float M_decoupl_d = 0.f, M_decoupl_q = 0.f;
	float M_d_or = 0.f, M_q_or = 0.f;
	float I_ds_r_hat_A = 0.f, I_qs_r_hat_A = 0.f;
	float I_ds_r_hat_B = 0.f, I_qs_r_hat_B = 0.f;
	float I_ds_r_hat_kp1_B = 0.f, I_qs_r_hat_kp1_B = 0.f;
	float v_ds_r_obs_B_FF = 0.f, v_qs_r_obs_B_FF =0.f;
	
	float Id_obs1=0.f,Id_obs2=0.f,Iq_obs1=0.f,Iq_obs2=0.f;
	float I_ds_r_err_obs_B = 0.f, I_qs_r_err_obs_B = 0.f;
		
	float Ias_kp1_B = 0.f;
	float Ibs_kp1_B = 0.f;
	float Ics_kp1_B = 0.f;
	float I_ds_err = 0.f, I_ds_err_p = 0.f, I_qs_err = 0.f, I_qs_err_p = 0.f;
	
		
	
//------ Stator flux observer (Discrete time)------// Added by Jaesuk
	float lambda_ds_r_B = 0.f, lambda_qs_r_B = 0.f;
	float lambda_ds_err = 0.f, lambda_qs_err = 0.f;
	float lambda_ds_r_hat_kp1 = 0.f, lambda_qs_r_hat_kp1 = 0.f;
	
	float v_flux_pd = 0.f, v_flux_id = 0.f, v_flux_pid = 0.f;
	float v_flux_pq = 0.f, v_flux_iq = 0.f, v_flux_piq = 0.f;
	
	float T_e_hat = 0.f, T_e_hat_kp1 = 0.f;
	float T_e_est = 0.f, T_e_est_kp1 = 0.f;	
	float T_em = 0.f;
	
	float temp1 = 0.f, temp2 = 0.f, temp3 = 0.f, temp4 = 0.f;
	float temp1_com = 0.f, temp2_com = 0.f, temp3_com = 0.f, temp4_com = 0.f;
	float lambda_ds_r_hat = 0.f, lambda_qs_r_hat =0.f;
	float c_d = 0.f, c_q =0.f;
	float lambda_ds_s_hat = 0.f,lambda_qs_s_hat = 0.f;
	
	float lambda_s_com = 0.1145f;
	float lambda_s = 0.f;	
	int Flux_Count = 1;
	
	float theta_flux = 0.f;
	float theta_flux2 = 0.f;
	float theta_flux_com = 0.f;
	
	float theta_err = 0.f;
	float theta_err_com = 0.f;
	float theta_r_err_B = 0.f;
	float lam_ds_1=0.f,lam_ds_2=0.f,lam_qs_1=0.f,lam_qs_2=0.f;
	float v_decoupl_d = 0.f, v_decoupl_q = 0.f;
	float Lam_ds_err = 0.f, Lam_qs_err = 0.f, Lam_ds_err_p = 0.f, Lam_qs_err_p = 0.f;
	
	float lambda_ds_r_delta = 0.f, lambda_qs_r_delta = 0.f;
	float v_ds_r_fb_i = 0.f, v_qs_r_fb_i = 0.f;
	float v_ds_r_fb = 0.f, v_qs_r_fb = 0.f;	 
				
//------Position observer------//
	float w_rm_hat = 0.f;
	float w_rm_hat_kp1 = 0.f;
	float theta_rm_hat_A = 0.f;
	float theta_rm_hat_kp1_A = 0.f;
	float w_rm_left = 0.f;
	float w_rm_left_kp1 = 0.f;
	float M_i_o_hat_km1 = 0.f;
	float T_d_hat_B = 0.f;
	float T_d_hat_A = 0.f;
	float theta_rm_hat_B = 0.f;
	float theta_rm_hat_kp1_B = 0.f;
	float theta_r_hat_A = 0.f;
	float theta_r_hat_B = 0.f;	
	float theta_err_hat = 0.f;
	float M_p_o = 0.f, M_i_o = 0.f;
	float T_hat = 0.f;	


	float w_r_hat_A = 0.f;
	float w_r_hat_B = 0.f;
	
//------ High Gain Nonlinear Observer --------------//
	float theta_r_HG = 0.f;
	float theta_r_HG_kp1 = 0.f;
	float theta_err_HG = 0.f;
	float w_r_HG_kp1 = 0.f;
	float w_r_HG = 0.f;
	float theta_r_deg = 0.f;
	float theta_r_hat = 0.f;
	float theta_err_deg = 0.f;
	//float theta_err = 0.f;
	float w_r_est = 0.f;
	
	//Observer Parameters
	float K1 = 5.f;
	float rho = 0.005f;
	

//------Encoder variables------//
	float last_encoder_count = 0.f, encoder_count = 0.f;
	float theta_rm_A = 0.f;
	float theta_r_A = 0.f;
	float encoder_count_offset = 0.f;
	float first_zp = 1.f;
		
	float w_rm_enc = 0.f;
	float theta_rm_A_last = 0.f;
		
	float theta_rm_B = 0.f;
	float theta_r_B = 0.f;
	
	
	
//------Motion Controller------//
	float T_com = 0.f;
	float w_rm_err = 0.f;
	float M_p_mc = 0.f;
	float M_i_mc = 0.f;
	float M_ii_mc = 0.f;
	float M_ii_mc_km1_temp=0.f;
	
	
	
	
	
	//Function variables
	float ref_frame = 0.f;			//1 = rotor reference frame, 0 = stationary reference frame
	float theta_ref_frame_A = 0.f;
	float theta_ref_frame_B = 0.f;

//------abc to dq transformation------//
	float alpha_A = 0.f, alpha_B = 0.f;
	float beta_A = 0.f, beta_B = 0.f;
	

//------Maximum torque per ampere LUT------//
	float T_index = 0.f;
	float percent = 0.f;
	int T_index_low = 0;
	int T_index_high = 0;
	
//------Maximum torque per ampere LUT (Machine B)------//
	float T_index_B = 0.f;
	float percent_B = 0.f;
	int T_index_low_B = 0;
	int T_index_high_B = 0;	
   	
//------Deadtime compensation------//
//Inverter A
	
	float T_a_comp = 0.f, T_b_comp = 0.f, T_c_comp = 0.f;
	
//Inverter B
	
	float T_a_comp_B = 0.f, T_b_comp_B = 0.f, T_c_comp_B = 0.f;
	
	//float v_LL_B = 0.f;
	
		
	
	//------Lq for motor B LUT------//
	float L_q_LUT[61]  =   {0.02200000000000f,0.02199709821429f,0.02198857142857f,0.02197468750000f,0.02195571428571f,0.02193191964286f,0.02190357142857f,0.02187093750000f,0.02183428571429f,0.02179388392857f,0.02175000000000f,0.02166297501880f,0.02150210165414f,0.02128107513158f,0.02101359067669f,
							0.02071334351504f,0.02039402887218f,0.02006934197368f,0.01975297804511f,0.01945863231203f,0.01920000000000f,0.01896104594427f,0.01871854464396f,0.01847540582043f,0.01823453919505f,0.01799885448916f,0.01777126142415f,0.01755466972136f,0.01735198910217f,0.01716612928793f,
							0.01700000000000f,0.01685042994652f,0.01671133262032f,0.01658124491979f,0.01645870374332f,0.01634224598930f,0.01623040855615f,0.01612172834225f,0.01601474224599f,0.01590798716578f,0.01580000000000f,0.01569163636364f,0.01558480808081f,0.01547963636364f,0.01537624242424f,
							0.01527474747475f,0.01517527272727f,0.01507793939394f,0.01498286868687f,0.01489018181818f,0.01480000000000f,0.01471190000000f,0.01462542222222f,0.01454063333333f,0.01445760000000f,0.01437638888889f,0.01429706666667f,0.01421970000000f,0.01414435555556f,0.01407110000000f,
							0.01400000000000f};
	
													
//------MTPA Look-up-table------//
	float MTPA_LUT[26][2]  =  { {0.00000000000000f, 0.00000000000000f}, {0.26612718445949f,-0.00930828833982f}, {0.53034829459410f,-0.03674995996848f}, {0.79100605118051f,-0.08087385120120f}, {1.04687572484668f,-0.13961793300780f}, {1.29816866943771f,-0.20816990821811f}, {1.54629558330359f,-0.28152884395254f},
								{1.79201078965160f,-0.35771528515228f}, {2.03477757592472f,-0.43728427631299f}, {2.27420471086090f,-0.52264007029083f}, {2.51201130436358f,-0.60905126028192f}, {2.74792705355734f,-0.69695957595573f}, {2.98125779627130f,-0.78670103767315f}, {3.21112360497046f,-0.87949388605779f},
								{3.43706837823438f,-0.97792955059742f}, {3.65992181908717f,-1.07966544262456f}, {3.88007910754510f,-1.18457585759470f}, {4.09830749886393f,-1.29185248056864f}, {4.31579568645255f,-1.39754736985674f}, {4.53157603089648f,-1.50462398336635f}, {4.74591519168393f,-1.61225232323212f},
								{4.95883568853943f,-1.72024821453322f}, {5.16990484526684f,-1.82993801949369f}, {5.37956652563658f,-1.94043693527992f}, {5.58803067850322f,-2.05109993778458f}, {5.79530109217718f,-2.16175787362496f} };		//From torque of 0 to 5 Nm in 0.1 increments {I_q,I_d} in the rotor reference frame
								
							

								
//test for parameter insensitivity

	float lambda_pm_b = C_LAMBDA_PM_B;
	
	float lam_ds_com = 0;
	float lam_qs_com = 0;
	float lambda_s_sqr = 0, lambda_dqs_mag = 0;
	float lambda_dqs_est = 0;
	float lambda_s_err2 = 0;

#ifdef _Over_Mod
//test for overmodulation
	float v_dc_o = 0.f;
	float v_dc_sqrt3 = 0.f;
#endif

	int overcount = 0;

		
//Saliency Tracking Observer
	

//Velocity Trapizoidal trajectory
	//int Trap_count = 1;
	//long int Flag_trap = 0;

//Chirp Signal used for Dynamic Stiffness Test
	int Flag_test = 0;

//Iron Loss Resistor Calculation
	float Pc = 0.f;   //Copper Loss 	
	float Pin = 0.f;

#ifdef _LMC
	//LMC (Loss Minimization Controller)
	float a1 = 0.f;
	float a2 = 0.f;
	float a3 = 0.f;
	float a4 = 0.f;
	float a5 = 0.f;
	float a6 = 0.f;
#endif

	float L_q_B = 0.f;
	float V_mag = 0.f;
	float B3 = 0.f;
	float B5 = 0.f;
/*********************************************************************************************************************/	
//Disturbance estimation for BEMF
//High bandwidth
//BEMF Tracking Observer
float sin_error_DIS = 0.f; //sinusoidal theta error calculated from the saliency BEMF
float theta_hat_BEMF_DIS = 0.f, theta_hat_BEMF_DIS_kp1 = 0.f; //estimated angle from the BEMF
float BT_o_p_DIS = 0.f, BT_o_i_DIS = 0.f;  //BEMF tracking observer controller
float BT_ko_DIS = 0.f, BT_kio_DIS = 0.f, BT_bm_o_DIS = 0.f; //BEMF tracking observer controller gains
float T_BT_DIS_hat = 0.f, T_BT_hat_o_DIS = 0.f, T_e_hat_BEMF = 0.f;
float w_left_BT_DIS = 0.f, w_hat_BT_DIS = 0.f, w_hat_BT_last_DIS = 0.f, w_e_hat_BT_DIS = 0.f; 
float theta_m_BT_hat_DIS = 0.f, theta_m_BT_hat_last_DIS = 0.f;
float theta_e_BT_hat_DIS = 0.f;
float theta_error_BT_DIS = 0.f;
/*********************************************************************************************************************/	
/*********************************************************************************************************************/
//Disturbance estimation Observer for HFI pulsating
/* Structures and defined types */

//Zero phase lag tracking observer
//Observer gain
//float ko_DIS = 0.f, kio_DIS = 0.f, bm_o_DIS = 0.f;
float theta_err_DIS = 0.f;      //estimated theta error
//Observer Controller
float ZT_o_p_DIS = 0.f, ZT_o_i_DIS = 0.f;
float T_DIS_hat = 0.f, T_hat_o_DIS = 0.f, T_e_hat_HFI = 0.f;
float w_left_DIS = 0.f, w_hat_DIS = 0.f, w_hat_last_DIS = 0.f; //estimated mechanical velocity
float theta_m_hat_DIS = 0.f, theta_m_hat_last_DIS = 0.f; //estimated mechanical angle
float theta_e_hat_DIS = 0.f; //estimated electrical angle
float theta_error_DIS = 0.f; //difference between the encoder and estimated angle for comparison
/*********************************************************************************************************************/	
/*********************************************************************************************************************/
//Saliency Tracking Observer for HFI pulsating
/* Structures and defined types */
static const float pi = 3.14159265f;
static const float rad2deg = 57.2957795f;
static const float deg2rad = 0.01745329f;
struct dq {float d,q;};				// Structure for referencing dq quantities
typedef struct dq dqdata[2];		// Array of 2 dq for filtering, etc.
static int current = 0, last = 1;
//Function Prototypes
static inline struct dq rotate(struct dq input, float angle); 
static inline void lpf(dqdata* input, dqdata* output, float bw); 	// First order low pass filter
static inline void hpf(dqdata* input, dqdata* output, float bw);  	// First order high pass filter

static inline void Mul2222(float RES2[2][2],float Mat2[2][2],float Mat22[2][2]); //2x2 Matrix with 2x2 Matrix multiplication//
static inline void Mul2221(float RES3[2],float Mat3[2][2],float Mat23[2]);    //2x2 Matrix with 2x1 Matrix multiplication//


//Estimated angle
float theta_A = 0.f, theta_B = 0.f; //using angle for the FOC, theta_r_A from encoder, theta_r_est_A from selfsensing
float theta_r_est_A = 0.f, theta_r_est_B = 0.f, theta_rm_est_A = 0.f;	
//SRFF
static dq Is_B = {0.f,0.f}; // current in stationary reference frame
static dqdata Ie_B = {{0.f,0.f},{0.f,0.f}}; // current in rotor reference frame
static dqdata Ie_h_B = {{0.f,0.f},{0.f,0.f}}; // current in rotor reference frame w/o fundamental component
static dqdata Ie_h_uf_B = {{0.f,0.f},{0.f,0.f}}; // current in rotor reference with multiplication of sinf(fovc_ph)
static dqdata Ie_h_f_B = {{0.f,0.f},{0.f,0.f}}; // filtered current in rotor reference
float I_err = 0.f;     // DC current error from the SRFF, used as input for tracking observer

static dqdata Is_B1 = {{0.f,0.f},{0.f,0.f}};
static dqdata Is_p = {{0.f,0.f},{0.f,0.f}};
static dqdata Is_n = {{0.f,0.f},{0.f,0.f}};
static dqdata Is_n1 = {{0.f,0.f},{0.f,0.f}};
static dqdata Is_hpf = {{0.f,0.f},{0.f,0.f}};
static dqdata Is_pf = {{0.f,0.f},{0.f,0.f}};
//Zero phase lag tracking observer
//Observer gain
//float ko = 0.f, kio = 0.f, bm_o = 0.f;
float theta_err_ZT = 0.f;      //estimated theta error
//Observer Controller
float ZT_o_p = 0.f, ZT_o_i = 0.f;
float T_FBS_hat = 0.f, T_hat_o = 0.f;
float w_left = 0.f, w_hat = 0.f, w_hat_last = 0.f; //estimated mechanical velocity
float theta_m_hat = 0.f, theta_m_hat_last = 0.f; //estimated mechanical angle
float theta_e_hat = 0.f; //estimated electrical angle
float theta_error = 0.f; //difference between the encoder and estimated angle for comparison
/*********************************************************************************************************************/	
/*********************************************************************************************************************/	
//Self-sensing based on BEMF
//Stator current filter in stationary reference frame
static dqdata Is_hat_B = {{0.f,0.f},{0.f,0.f}}; //Estimated stator current in stationary reference frame
static dq Is_err = {0.f,0.f}; // Estimated stator current error in stationary reference frame
float SC_o_d_p = 0.f, SC_o_q_p = 0.f, SC_o_d_i = 0.f, SC_o_q_i = 0.f,SC_o_d = 0.f, SC_o_q = 0.f; //Stator current observer controller
float SC_ko = 0.f, SC_kio = 0.f; //Stator Current observer controller gains
float C_B_S = 0.f, C_A_S = 0.f;
static dq E_sal_B = {0.f,0.f}; // Saliency back EMF
int temp;  //variable for swap the index
//BEMF Tracking Observer
float E_sal_mag = 0.f; //magnitude of the saliency BEMF
float sin_error = 0.f; //sinusoidal theta error calculated from the saliency BEMF
float theta_hat_BEMF = 0.f, theta_hat_BEMF_kp1 = 0.f; //estimated angle from the BEMF
float BT_o_p = 0.f, BT_o_i = 0.f;  //BEMF tracking observer controller
float BT_ko = 0.f, BT_kio = 0.f, BT_bm_o = 0.f; //BEMF tracking observer controller gains
float T_BT_FBS_hat = 0.f, T_BT_hat_o = 0.f;
float w_left_BT = 0.f, w_hat_BT = 0.f, w_hat_BT_last = 0.f, w_e_hat_BT = 0.f; 
float theta_m_BT_hat = 0.f, theta_m_BT_hat_last = 0.f;
float theta_e_BT_hat = 0.f;
float theta_error_BT = 0.f;
/*********************************************************************************************************************/	

/*********************************************************************************************************************/	
//Adaptive Observer for HFI
float theta_err_hat1 = 0.f, theta_m_hat1 = 0.f, theta_m_hat1_last = 0.f, theta_e_hat1 = 0.f, theta_m_APT_hat = 0.f, theta_e_APT_hat = 0.f;  //angle error
float APT_o_p = 0.f, APT_o_i = 0.f, APT_o_b = 0.f;//Combined Motion Observer controller
//float APT_ko = 0.f, APT_kio = 0.f, APT_bm_o = 0.f; //Combined Motion Observer controller gain
float T_APT_hat = 0.f, T_APT_hat_o = 0.f;
float L = 0.f, Q0 = 0.f, R = 5.f;      //initial value of P0
float w_hat_APT = 0.f,w_hat_APT_last = 0.f, w_e_hat_APT = 0.f, w_err_hat = 0.f, w_m_APT_hat = 0.f, w_e_APT_hat = 0.f,w_left_APT_hat = 0.f;
float theta_error_APT = 0.f;
float x_k_hat[2] = {0,0}, x_kp1[2] = {0,0},x_kp1_hat[2] = {0,0},P_kp1_temp1[2] = {0,0},P_kp1_temp2[2][2] = {{0,0},{0,0}};
float P_kp1[2][2] = {{0,0},{0,0}}, P_hat_kp1[2][2] = {{0,0},{0,0}},P_temp1[2][2] = {{0,0},{0,0}}, P_temp2[2][2] = {{0,0},{0,0}};
float K_temp1[2] = {0,0}, K_temp2 = 0.f,K_temp4[2] = {0,0}, K_temp3 = 0.f;
float Q_set = 0,P_set = 0,R_set = 0;
/*********************************************************************************************************************/	

/*********************************************************************************************************************/	
//Cascaded Combined Motion Observer (Low Speed: HFI, High Speed: BEMF)
float theta_err_hat2 = 0.f, theta_m_hat2 = 0.f, theta_m_hat2_last = 0.f, theta_e_hat2 = 0.f;  //angle error
float CM_o_p = 0.f, CM_o_i = 0.f;//Combined Motion Observer controller
float CM_ko = 0.f, CM_kio = 0.f, CM_bm_o = 0.f; //Combined Motion Observer controller gain
float T_CM_hat = 0.f, T_CM_hat_o = 0.f;
float w_left_CM = 0.f,w_hat_CM = 0.f,w_hat_CM_last = 0.f, w_e_hat_CM = 0.f;
float theta_error_CM = 0.f;
float I_d_com = 0.f;

/*********************************************************************************************************************/	
//XCSDataLink
	Command command;
	UnionData data;
	volatile unsigned cmd;											
	// Command data for data linker
	volatile float cmd_data;	
	
	int dataset = 0;
	
//Data Logging (Average Value)
	int Ave_Count = 1;
	float Torque_Ave = 0;
	float Lambda_Ave = 0;
	float w_rm_hat_Ave = 0;
	float Pc_Ave = 0;  //Copper Loss Calculation
	float Id_hat_Ave = 0;
	float Iq_hat_Ave = 0;
	float w_rm_set = 0;
	float T_set_B = 0;
	float lambda_set = 0;

	float v_ds_r_com_B_BC = 0;
	float v_qs_r_com_B_BC = 0;
	
	float Rs = 0;
	
	float Ap1 = 0;
	float Dp1 = 0;
	float Cp1 = 0;
	float m_p = 0;
	float b_p = 0;
	float P_loss_com = 0;
	float P_loss_set = 0;
	float P_loss = 0;
	float P_loss1 = 0;
	float P_loss_err = 0;
	float v_ds_r_com_B_P_BC = 0;
	float v_qs_r_com_B_P_BC = 0;
	float v_ds_r_com_B_P = 0;
	float v_qs_r_com_B_P = 0;
	float z_d_P = 0;
	float z_q_P = 0;
	
	float ovm_ratio = 0;
	float P_loss_f = 0;  //filtered copper loss
	float P_loss_km1 = 0;
	float P_loss_f_km1 = 0;
	float T_est_f = 0;
	float T_est_f_km1 = 0;
	float T_est_km1 = 0;
	
	//Low Pass Filter Coefficients
	float n_P = 0;
	float d_P = 0;
	float n_T = 0;
	float d_T = 0;
	
	float Trm_Count = 0;
	//Voltage Hexagon
	float V_Hex = 0;
	
	float b_p_1 = 0;
	float b_p_2 = 0;
	float b_p_3 = 0;
	float a_p_1 = 0;
	float c_p_1 = 0;
	float c_p_2 = 0;
	float c_p_3 = 0;
	float C_P = 0;
	
	float Flag1 = 0;
	float Flag2 = 0;
	float Count_p = 0;
	float P_loss_km11 = 0;
	
	float theta_lambda = 0;
	
	float bb = 0;
	float bb1 = 0;
	
	float v_ds_r_com_B_km1 = 0;
	float v_qs_r_com_B_km1 = 0;
	float slew_rate = 0;
	
	float Flag_Inj = 0;
	
	//different flux trajectory in one fundamental circle
	float Flag_Traj = 0;
	//Flux Trajectory counter  80 Hz(250 rad/s) ---- Traj_count = 125,  40 Hz (125 rad/s) ------- Traj_count = 250
	float Traj_count = 1;
	int sign1 = 1;
	float lambda_c = 0;
	
	float test_sqrt = 0;
	float lam_ds_2_k = 0;
	float lam_qs_2_k = 0;
	
	float T_e_hat_A = 0;
	int ENC_Count = 0;
	
	int Align_Count = 0;
	
	//Neareast Six Step Overmodulation
	float v_dqs_s_star_mag = 0;
	float alpha_ovm = 0;
	float mu = 0;
	float ovm = 0;
	float v_ds_r = 0;
	float v_qs_r = 0;
	float v_ds_r_B = 0;
	float v_qs_r_B = 0;
	
	//Optimal Maximum Torque Calcuation
	
	float A_MT_1 = 0;
	float B_MT_1 = 0;
	float root_MT = 0;
	
	int Limit_Count = 0;
	
	float v_ds_r_com_B1 = 0;
	float v_qs_r_com_B1 = 0;
	
	float I_ds_r_B1 = 0, I_qs_r_B1 = 0;
	int angle_sign = 1;
	int Axis_Count = 0;
	int Flag_d = 0;
	int Flag_Alt = 1;
	int db_count = 1;
	int Flag_T = 0;

	float T_rate = 2.26;
	
	//Dynamic Loss Coefficient
	//float Ke = 0.0027f;
	//float Kh = 6.2f;
	float Ke = 0.0037f;
	float Kh = 0.5086f;
	
	float lambda_d_km1 = 0.f;
	float lambda_q_km1 = 0.f;
	float Pc_kp1 = 0.f;
	float Ph_kp1 = 0.f;
	float Pe_kp1 = 0.f;
	float Pdyn_kp1 = 0.f;
	float H1 = 0.f;
	float H2 = 0.f;
	float Ah = 0.f;
	float Ae = 0.f;
	int Flux_Traj_Cnt = 1;
	float Wdyn = 0.f;
	float Wout = 0.f;
	
	
	float lam_ds_kp1_1 = 0.f;
	float lam_qs_kp1_1 = 0.f;
	float Pc_kp1_1 = 0.f;
	float Ph_kp1_1 = 0.f;
	float Pe_kp1_1 = 0.f;
	float Pdyn_kp1_1 = 0.f;
	float H1_1 = 0.f;
	float H2_1 = 0.f;
	float lam_dqs_est_1 = 0.f;
	
	float v_ds_r_com_f_B = 0.f;
	float v_qs_r_com_f_B = 0.f;
	
	float T_com_hf = 0.1f;
	float lam_com = 0.f;
	//float lam_com1 = 0.1145f; //(100 rad/s)
	float lam_com1 = 0.109f;   //(400 rad/s)
	//float lam_com1 = 0.1126f; //(200 rad/s)
	
	float Pin1 = 0.f;
	
	float Phd = 0.f;
	float Phq = 0.f;
	float Pe = 0.f;
	
	float lam_com_hf = 0.0f;
	
	float Vs = 0.0f;
	
	float vds_I_DID = 0.0f;
	float vqs_I_DID = 0.0f;
	
	float vds_PI_DID = 0.0f;
	float vqs_PI_DID = 0.0f;
	
	int Flag_PI = 2U;
	
	float wr_com_hf = 0.0f;
	
	float Lq = 0.0f;
	float Ld = 0.0f;
	float Ld_obs = 0.0f;
	float Lq_obs = 0.0f;
	float lambda_pm_obs = 0.0f;
	
	float Ld_set = 0.0f;
	float Lq_set = 0.0f;
	float lambda_pm_set = 0.0f;
	float lambda_pm = 0.0f;
	
	float T_e_hat_kp11 = 0.0f;
	float lambda_qs_r_B1 = 0.0f;
	float lambda_ds_r_B1 = 0.0f;
	
	//flux model for DID stator flux observer
	float vds_s_I_DID = 0.0f;
	float vqs_s_I_DID = 0.0f;
	
	float delta_lambda_ds_s = 0.0f;
	float delta_lambda_qs_s = 0.0f;
	
	static dqdata delta_lambda_dqs_s = {{0.f,0.f},{0.f,0.f}}; //delta flux before high pass filter in stator ref. frame
	static dqdata delta_lambda_dqs_s_f = {{0.f,0.f},{0.f,0.f}}; // delta flux after high pass filter in stator ref. frame
	
	static dqdata delta_lambda_dqs_r = {{0.f,0.f},{0.f,0.f}}; //delta flux before low pass filter in rotor ref. frame
	static dqdata delta_lambda_dqs_r_f = {{0.f,0.f},{0.f,0.f}}; // delta flux after low pass filter in rotor ref. frame
	
	float delta_lambda_ds_r = 0.f;
	float delta_lambda_qs_r = 0.f;
	
	float theta_AB = 0.f;
	
	
	float BW_hpf = 0.f;
	float BW_lpf = 0.f;