/*************************************************************************/
/*						DBDTFC Control Program - main.h					 */
/*						v4.0											 */
/*						09/13/2011										 */
/*						Zachary Hurst, Brian Bradley					 */
/*************************************************************************/
// DP_v3_T_load used to do lock the rotor test, motion control on load side


// Libaries Necessary to make code work
#include"hardware.h"
#include"init_pwm.h"
#include"ENCOD_A.h"
#include <irq.h>
#include <math.h>
#include "XCSDataLink.h"
//#include <terminal.h>
#include <stdlib.h>
#include <I2C.h>
#include<shared/extract.h>
#include<shared/nop.h>
#include<shared/lib_clip.h>
#include<terminal.h>
#include <AixScope.h>
#include "ComplexVar.h"
//#include <Drive_Cycle.h>
extern void menu_init ();
extern void menu_serve();
const unsigned comm_serve_N = 1;
volatile unsigned comm_serve_count = 0;

/*	Definition of constants	*/
	static const float pi				= 3.14159265f;
	static const float two_pi 			= 2.f*pi;
	static const float two_pi_over_3	= (2.f*pi)/3;
	static const float pi_over_2 		= pi/2;
	static const float pi_over_3 		= pi/3;
	static const float pi_over_6		= pi/6;
	static const float pi_over_12		= pi/12;
	static const float five_pi_over_6	= (5.f*pi)/6;
	static const float seven_pi_over_6	= (7.f*pi)/6;
	static const float three_pi_over_2	= (3.f*pi)/2; 
	static const float one_third 		= 0.333333333f;
	static const float two_thirds 		= 0.66666666f;
	static const float divroot3 		= 0.5773502663f;
	static const float sqrt3 			= 1.7320508075688f;
	static const float one_over_sqrt3 	= 0.57735f;
	static const float sqrt3_over_3 	= sqrt3/3;
	static const float sqrt3_over_2 	= sqrt3/2;
	static const float deg2rad 			= 0.01745329254f;
	static const float rad2deg 			= 57.2957795f;
	static const float rad_s2rpm 		= 9.54929659f;
	static const float sin_2pi_by_3 	= 0.866025403784439f;
	static const float sqrt2_over_2		= 0.707106781186548f;
	
/* Structures and defined types */
	// Structure for referencing dq quantities
	struct dq {float d,q;};

	struct dq 	v_ref;
	volatile float v_ref_mag;
	// switching frequency of the PWM
	const float pwm_frequency = 20000.0f;
	//volatile float PWM_frequency_divisor = 1;
	//volatile int count_PWM = 1;
	//const float pwm_frequency = 1500.f;//10000.f;
	// switching period
	volatile float dt = 1/pwm_frequency;

	// motion loop frequency (Multiple of PWM)
	//const float enc_frequency = 2000.0f;
	//const float enc_frequency = 0.015f;//;0.015f;//0.1f;
	// communications update frequency
	const float link_frequency = 10000.f;
	// software generated deadtime
	const float pwm_deadtime = 0.2e-6f;

	// PWM compensation(need to change back to volatile if we need to improve compensation)
	// Tuned dead time comp value
	const float pwm_t_comp = 0.03f;
	// Tuned dead time comp value
	const float pwm_inv_comp = 1.5f;
	// Holding var for compenation time calc
	volatile float pwm_t_phase_comp_ref = 0.0f;
	// Modulation index dead time compensation value
	volatile float pwm_t_phase_comp = 0.0f;
	// Inverter FWD voltage drop of power silicon
	const float v_fwd_drop = 0.0f;

/* Menu Updating */
	// Number of link cycles to decimate before updating menu routine
	const int menu_decimate = 50;
	// Counter var for menu routine
	volatile int menu_count = 0;
	
/* Motor constants */
	// Number of pole pairs in the machines
	const int pole_pairs = 4;									

/* Drive States */
	enum{
	    READY,
	    STARTUP,
	    RUN,
	    STOP,
	    INIT
	};

	// Drive flag
	volatile   int drive_flag = STOP;//Set to RUN minhao
	// Number of samples to spend in startup state
	volatile   int startup_count = 10000;
	// Number of samples to spend in run
//	volatile   int run_count = 0;
	// Number of samples to spend in init - zeroing out current sensors
	volatile   int init_count = 833;
	

/* System Protection */
	// Max  line current (A)
	volatile float max_current 	= 10.0f;
	// Max dc link voltage (V) 
	volatile float max_voltage 	= 60.0f;
	// Maximum shaft speed (rad/s)
	volatile float omega_max 	= 628.318f; // Value equivalent to 6000 RPM
	// Divisor of sampling frequency for lower position sampling for higher speed resolution
	const float delta_theta_mult = 10.0f;
	// Largest allowable change in position
	volatile float delta_theta_max = omega_max*delta_theta_mult/pwm_frequency;

	ComplexVar I_dq_s_A;
	ComplexVar I_dq_s_f_A;
	ComplexVar I_dq_c_A;
	float E_norm_d_s_A;
	float E_norm_q_s_A;
	float E_norm_d_s_M;
	float E_norm_q_s_M;


/* Function Prototypes */
	// Encoder Sampling, 400kHz
	void enc_isr(irq_param);
	// Current regulator loop at 20 kHz
	void control_ISR (irq_param);
	// Data transfer loop at 2 kHz
	void link_isr(irq_param);
	// Menu Update Loop at 50Hz
	void menu_isr(irq_param);
	// Function runs once on startup
	void init(void);
	// Runs continuously in background
	void main_loop(void);
	// Used for rotating vector structures
	static inline struct dq rotate(struct dq input, float angle);
	static inline float phase_in_two_pi(float Vqs, float Vds);



/*   Logging variables   */
	// Structure used for XCS data linker for Matlab
	struct Command { 											
	    unsigned ID; 
	    unsigned Data; 
	};
	// Structure used for XCS data linker for Matlab
	union UnionData { 											
	    unsigned uint; 
	    float f; 
	};
		
	// Datalinker object creation
	DataLink::XCSDataLink link;
	// Total number of samples per signal to capture
	const    int log_num_samples = 2.0f*pwm_frequency;//2250;//50000;//2250;//1030;	//(Max around )							
	// Total number of signals to capture
	const    int log_num_channels = 12;//2;//30;								
	// Max length of data log
	volatile int log_data_length;									
	// Counter for data record length
	volatile int log_dump_count =0;									

	// Flag for turning on data logging
	volatile int flag_rec;											
	// Number of samples to decimate for logging
	volatile int log_sample_decimate=1;								
	// Counter for decimating count
	volatile int log_decimate_count=1;								
	// Bank Selection for Data logging
	volatile int count = 0;
	volatile int count_omega = 1;
	
	#ifdef _SQUARE_TORQUE
		volatile int count_square = 1;
		volatile int k_square;	
		bool 		 square_torque_flag = false;	
	#endif
	
	#ifdef _SINE_WAVE
		volatile int 	count_sine 		= 1;
		volatile float	sine_freq 		= 4.0f;
		volatile float  sine_amp 		= 1.0f;
		volatile float  sine_offset 	= 0.0f;//1.5f;
		bool		 	sine_wave_flag  = false;
		volatile float  sine_q;// 			= 0.0f;
		volatile float  sine_d;// 			= 0.0f;
	#endif
	
	volatile float dataset = 1.0f;
	// Command Variable for data linker
	Command      command;											
	// Data structure for data logger
	UnionData    data;												
	// Command for data linker
	volatile unsigned cmd;											
	// Command data for data linker
	volatile float cmd_data;										
	// Create large spot in memory to store off logging data
	_Pragma( "section( \"/NO_INIT seg_SDRAM_NOINIT\" )" )
	//_Pragma( "section( \"seg_SDRAM\" )" )
	//	_Pragma( "section( \"seg_SRAM\" )" )
	volatile float value_dump[log_num_samples][log_num_channels];
	//_Pragma( "section( \"seg_SRAM\" )" )



// Command Values for using Data Linker to Matlab
// See XLS spreadsheet for details
	enum{
	    CMD_NOP							=1,	
	    CMD_STOP						=10,
	    CMD_START						=11,
	    CMD_READY						=12,
	    CMD_SEND_DATA					=21,
		CMD_SEND_DATA_SIZE				=22,
		CMD_SEND_NUM_CHAN				=23,
		CMD_REC_FLAG 					=24,
		CMD_REC_DECIMATE				=25,
		CMD_DATASET						=26, 
		CMD_OMEGA_STAR_1 				=101,
		CMD_BA_1 						=102,
		CMD_KSA_1						=103,
		CMD_KISA_1						=104,
		CMD_BA2_1						=105,
		CMD_JA_1						=106,
		CMD_DRIVECYCLE_JA_1				=107,
		CMD_DRIVECYCLE_DRAG_1			=108,
		CMD_DRIVECYCLE_RESISTANCE_1		=109,
		CMD_FOC_FLUX_STAR_1				=162,
		CMD_OMEGA_SELECT_2				=201,
		CMD_BA_2						=202,
		CMD_KSA_2						=203,
		CMD_KISA_2						=204,
		CMD_OMEGA_REF_2					=210,
		CMD_OMEGA_SINE_OFFSET_2			=211,
		CMD_OMEGA_SINE_AMP_2			=212,
		CMD_OMEGA_SINE_FREQ_2			=213,
		CMD_OMEGA_SLEW_2 				=214,
		CMD_TORQUE_SLEW_2				=215,
		CMD_TORQUE_SINE_OFFSET_1		=220,
		CMD_TORQUE_SINE_AMP_1			=221,
		CMD_TORQUE_SINE_FREQ_1			=222,
		CMD_LOAD_TRAJ_SELECT_1			=225,
		CMD_CYCLICAL_FREQ				=226,
		CMD_DYN_LOAD_NCYCLES            =227,
		CMD_TORQUE_MOD_SELECT_2			=240,
		CMD_TORQUE_MOD_SELECT_1			=241,
		CMD_VS_AMP_2					=251,
		CMD_VS_FREQ_2					=252,
		CMD_VS_AMP_1					=253,
		CMD_VS_FREQ_1					=254,
		CMD_FOC_REG_P_2					=260,
		CMD_FOC_REG_I_2					=261,
		CMD_FOC_FLUX_STAR_2				=262,
		CMD_FOC_IQS_SINE_WAVE_2			=263,
		CMD_FOC_IQS_SINE_WAVE_FREQ_2    =264,
		CMD_FOC_IQS_SINE_WAVE_AMP_2     =265,
		CMD_DB_FLUX_SELECT_2			=270,
		CMD_DB_FLUX_STAR_2				=271,
		CMD_DB_FLUX_TAU_2 				=272,
		CMD_DB_FLUX_SLEW_2 				=273,
		CMD_DB_TORQUE_STAR_SQURE_2		=274,
		CMD_INV_COMP_REF_T_2			=280,
		CMD_INV_COMP_2					=281,
		CMD_INV_V_FWD_DROP_2			=282,
		CMD_SS_COLLECT_FLAG				=300,
		CMD_DYN_MACRO_CMD				=305,
		CMD_DYN_MACRO_LAMBDA_STAR		=306,
		CMD_DYN_MARCO_PRETRIG			=307,
		CMD_DYN_MARCO_OMEGA_STAR 		=308,
		CMD_DYN_MARCO_TORQ_CMD 			=309,
		CMD_DYN_MARCO_TORQ_SLEW 		=310,
		CMD_DYN_MARCO_TORQ_OFFSET 		=311,
		CMD_DYN_MARCO_TORQ_CMD_NXT 		=312,
		CMD_LAMBDA_SINE_OFFSET_2		=313,
		CMD_LAMBDA_SINE_AMP_2			=314,
		CMD_LAMBDA_SINE_FREQ_2			=315,
		CMD_FOC_TORQUE_STAR_1           =316,
		CMD_FOC_TORQUE_STAR_2           =317,
		CMD_OMEGA_STAR_2                =318,
		CMD_FOC_IQS_E_STAR_1			=319,
		CMD_FOC_IDS_E_STAR_1			=320,
		CMD_FOC_IQS_E_STAR_2			=321,
		CMD_FOC_IDS_E_STAR_2			=322,
		CMD_PWM_FREQUENCY_2             =500,
		CMD_NOISE						=600,
		CMD_CHIRP_DIRECTION             =601,
		CMD_CURRENT_OBS_LOW_2			=602,
		
	};





/* Voltage Vars*/
	// PWM Commanded Voltage (V)
	volatile float v_a_1;	
	volatile float v_b_1;	
	volatile float v_c_1;
	volatile float v_a_2;
	volatile float v_b_2;	
	volatile float v_c_2;
	
	// volatile float v_ab_2_ad;
	// volatile float v_ca_2_ad;	
	// volatile float v_bc_2_ad;
	// volatile float v_a_2_measured;
	// volatile float v_b_2_measured;	
	// volatile float v_c_2_measured;

	 //Voltage measured Vars
	volatile float v_a_2_measured;
	volatile float v_b_2_measured;
	volatile float v_c_2_measured;
	
	//struct   dq    v_dqs_sense_e_2;
			
	volatile float v_max_2;	
	volatile float v_min_2;	
	volatile float v_n_2 = 0.0f;
	// DC Link Voltages
	volatile float v_dc_link;	
	volatile float v_dc_link_filtered=0.0f;
	// Anti-windup variables
	volatile float v_q_remnant_1;
	struct 	 dq    v_pre_sat_1;
	volatile float v_q_remnant_2;
	struct 	 dq    v_pre_sat_2;
	// Modulation index
	volatile float v_a_pwm_1;	
	volatile float v_b_pwm_1;	
	volatile float v_c_pwm_1;	
	volatile float v_a_pwm_2;		
	volatile float v_b_pwm_2;
	volatile float v_c_pwm_2;

/* Current Vars */
	// Line current (A)
	volatile float i_a_1;
	volatile float i_b_1;	
	volatile float i_c_1;
	volatile float i_a_2;			
	volatile float i_b_2;										
	volatile float i_c_2;									
	volatile float i_a_inv_2;								
	volatile float i_b_inv_2;								
	volatile float i_c_inv_2;									
	volatile float i_a_offset_1			= -0.0338727f;//1.0f/3.0f;//-0.0327f;			
	volatile float i_b_offset_1			= -0.0327073f;//1.0f/3.0f;//-0.0338f;				
	volatile float i_c_offset_1			= 0.0f;//-0.0157f;			
	volatile float i_a_offset_2			= -0.0294593023f;//-0.8235f;//-0.1064f;//-0.0220f; 
	volatile float i_b_offset_2			= 0.0017f;//-0.3805f;//-0.3703f;//-0.0130f; 
	volatile float i_c_offset_2			= 0;//0.0221f;// 0.0110f;
	volatile float v_ab_offset_2        = 0;//-2.2748f;//-24.2358f;
	volatile float v_ca_offset_2        = 0;//-2.2224f;//-3.0113f;//-23.2118f;
	volatile float v_bc_offset_2        = 0;//-2.2563f;//-3.0208f;//-23.9803f;
	volatile float i_a_inv_offset_2		= 0.0f;// 0.4695f;
	volatile float i_b_inv_offset_2		= 0.0f;//-0.0156f;	
	volatile float i_c_inv_offset_2		= 0.0f;//-0.0201f;
	
	// dq axis statonary frame current (A)
	struct      dq i_dqs_s_1;
	struct      dq i_dqs_s_2;
	//struct      dq i_dqs_s_2_km1;
	//struct      dq i_dqs_s_2_eff;

/* Motion  Vars */
	// Total encoder counts per rev (4x Quad)
	const 	 int   total_enc_cnt			= 4000;
	// Current encoder count value
	volatile  int   enc_cnt 	  	= 0;
	volatile  int   enc_cnt_0       = 0;
	volatile  int	enc_cnt_1		= 0;
	volatile  int   enc_cnt_0_1     = 0;
	// Number of Encoder counts since last interrupt 					
	volatile int   delta_cnt 				= 0;
	volatile int   delta_cnt_1				= 0;
	// Delta count for omega bar calc					
	volatile int   delta_cnt_w_r;	
	// Pervious encoder count value								
	volatile int   last_cnt 				= 0;
	volatile int   last_cnt_1				= 0;
	// Flag used for decimating omega bar clac					
	volatile int   motion_flag				= 0;
	// Average Speed (rad/s)					
	volatile float w_bar;
	// Shaft position (rad)											
	volatile float theta;
	volatile float theta_0;
	
	volatile float theta_1;
	volatile float theta_0_1;
	// (k-1)Shaft position (rad)											
	volatile float theta_km1;
	volatile float theta_km1_1;
	// Theta difference for encoder calc 	
	volatile float delta_theta;
	volatile float delta_theta_1;
	// RPM of shaft for display  									
	volatile float rpm;												

// Machine 1 Motion Observer
	// Position error (rad)
	volatile float theta_err_1				=0.0f;
	volatile float theta_err_1_km1			=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_1		=0.0f;					
	// Torque command feedforward (Nm)
	volatile float t_cff_obsr_1				=0.0f;
	
	/* Gains for running in 10Khz	*/
						
	// Observer damping 
	const float bo_1					= 0.006730205152075f;//0.012669774663485f;//0.0079f;//0.0142;//=0.0079f;					
	// Observer stiffness
	const float kso_1					= 0.228311916887107f;//0.4963f;//144.6118;//=0.4963f;				
	// Observer integral stiffness
	const float kiso_1					= 1.175796677469615f;//5.0309f;//727600;//=5.0309f;
	
	
	/* Gains for running in 1500hz	*/
		/*// Observer damping 
		volatile float bo_1						=13.5f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;					
	// Observer stiffness
	volatile float kso_1					=17349.0f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;				
	// Observer integral stiffness
	volatile float kiso_1					=42568.0f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;	
	*/
	
				
	// Estimated theta position for next loop
	volatile float theta_hat_kp1_1			=0.0f;					
	// Estimated theta position for current count
	volatile float theta_hat_k_1			=0.0f;					
	// Estimated test stand shaft inertia (Kg*m^2)
	const float j_p_hat_1				=0.000019f;	//=0.066f;//=0.044f; //=0.022f;				
	// Inverse of estimated shaft inertia
	const float j_p_hat_inv_1			=1.0f/j_p_hat_1;		
	// Observer sample time
	volatile    float t_obsr_1			= delta_theta_mult/pwm_frequency;	
	// Estimated shaft speed
	volatile float omega_l_hat_kp1_1 		=0.0f;					
	// Estimated shaft speed
	volatile float omega_l_hat_k_1   		=0.0f;					
	// Estimated shaft speed
	volatile float omega_r_hat_kp1_1 		=0.0f;					
	// Estimated shaft speed
	volatile float omega_r_hat_k_1   		=0.0f;					
	// Estimated average shaft speed
	volatile float omega_bar_hat_kp1_1 		=0.0f;					
	// Estiamted average shaft speed
	volatile float omega_bar_hat_k_1   		=0.0f;					
	// Torq being made up by the observer (Not including tcff)
	volatile float torq_obsr_1;										
	// Acceleration Estimate for Active Inertia
	volatile float omega_dot_kp1_1			=0.0f;					
	// Acceleration Estimate for Active Inertia (Filtered)
	volatile float omega_dot_filt_kp1_1     = 0.0f;
	
	volatile float omega_encoder			=0.0f;
	volatile float omega_encoder_1			=0.0f;
	volatile float omega_enc_filtered;
	volatile float theta_filtered;

	
// Machine 2 Shaft Torque Observer
	// Position error (rad)
	volatile float theta_err_2				=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_2		=0.0f;					
	// Torque command feedforward (Nm)
	volatile float t_cff_obsr_2				=0.0f;					
	// Observer damping 
	const float bo_2					=0.006730205152075f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;100Hz					
	// Observer stiffness
	const float kso_2					=0.228311916887107f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;5Hz				
	// Observer integral stiffness
	const float kiso_2					=1.175796677469615f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;1hz	
	
	/* Gains for running in 1500hz	*/
	// Observer damping 
	//	volatile float bo_2						=13.5f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;					
	// Observer stiffness
	//	volatile float kso_2					=17349.0f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;				
	// Observer integral stiffness
	//	volatile float kiso_2					=42568.0f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;	
	
				
	// Estimated theta position for next loop
	volatile float theta_hat_kp1_2			=0.0f;					
	// Estimated theta position for current count
	volatile float theta_hat_k_2			=0.0f;					
	// Estimated test stand shaft inertia (Kg*m^2)
	const float j_p_hat_2				=0.000019f;//=0.066f;//=0.044f; //=0.022f;				
	// Inverse of estimated shaft inertia
	const float j_p_hat_inv_2			=1.0f/j_p_hat_1;			
	// Estimated shaft speed
	volatile float omega_l_hat_kp1_2 		=0.0f;					
	// Estimated shaft speed
	volatile float omega_l_hat_k_2   		=0.0f;					
	// Estimated shaft speed
	volatile float omega_r_hat_kp1_2 		=0.0f;					
	// Estimated shaft speed
	volatile float omega_r_hat_k_2   		=0.0f;					
	// Estimated average shaft speed
	volatile float omega_bar_hat_kp1_2 		=0.0f;					
	// Estiamted average shaft speed
	volatile float omega_bar_hat_k_2   		=0.0f;					
	// Torq being made up by the observer (Not including tcff)
	volatile float torq_obsr_2;										
	// Acceleration Estimate for Active Inertia
	volatile float omega_dot_kp1_2			=0.0f;					
	// Acceleration Estimate for Active Inertia (Filtered)
	volatile float omega_dot_filt_kp1_2     = 0.0f;

// Machine 1 Shaft Torque Cascaded Acceleration Observer
/*
	// Position error (rad)
	volatile float theta_err_2a				=0.0f;
	// Velocity Error (rad/s)
	volatile float omega_hat_error_2a		=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_2a		=0.0f;					
	// Torque command feedforward (Nm)
	volatile float t_cff_obsr_2a				=0.0f;					
	// Observer damping 
	volatile float bo_2a					=7.1f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;					
	// Observer stiffness
	volatile float kso_2a					=250.f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;				
	// Observer integral stiffness
	volatile float kiso_2a					=1300.f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;				
	// Estimated theta position for next loop
	volatile float theta_hat_kp1_2a			=0.0f;					
	// Estimated theta position for current count
	volatile float theta_hat_k_2a			=0.0f;					
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_drive_hat_2a				=0.011f;//=0.066f;//=0.044f; //=0.022f;				
	// Inverse of estimated shaft inertia
	volatile float j_drive_hat_inv_2a			=1.0f/j_drive_hat_2a;			
	// Estimated shaft speed
	volatile float omega_hat_kp1_2a 		=0.0f;					
	// Estimated shaft speed
	volatile float omega_hat_k_2a   		=0.0f;					
	// Estimated average shaft speed
	volatile float omega_bar_hat_kp1_2a 		=0.0f;					
	// Estiamted average shaft speed
	volatile float omega_bar_hat_k_2a   		=0.0f;					
	// Torq being made up by the observer (Not including tcff)
	volatile float torq_obsr_2a;										
	// Acceleration Estimate for Active Inertia
	volatile float omega_dot_kp1_2a			=0.0f;					
	// Acceleration Estimate for Active Inertia (Filtered)
	//volatile float omega_dot_filt_kp1_2a     = 0.0f;
*/		
	
// Machine 1 Motion Controller
	// Desired machine shaft speed (rad/s)
	volatile float omega_star_kp1_1			=0.0f;					
	// Velocity error
	volatile float omega_err_kp1_1			=0.0f;					
	// Error accumulator for Ksa
	volatile float omega_err_kp1_accum1_1	=0.0f; 					
	// Second error accumulator for Kisa
	volatile float omega_err_kp1_accum2_1	=0.0f;					
	// Troque command feedforward value (Nm)
	volatile float t_cff_sfb_1				=0.0f;					
	// SFb motion controller loop time
	volatile    float t_sfb_1			=  delta_theta_mult/pwm_frequency;			
	// Active Inertia
	volatile float ja_1						=0.0f;	
	
	// Desired machine shaft reference for sine trajectory
	volatile float tem_star_sin_amp_2		=0.0f;
	// Desired machine shaft reference frequency for sine trajectory
	volatile float tem_star_sin_freq_2	=0.0f;
	// Desired machine shaft reference offset for sine trajectory
	volatile float tem_star_sin_offset_2	=0.0f;				
	
	
	// Vehicle Mass for Drivecycle
	volatile float drivecycle_ja_1			= 0.0f;
	// Drivecycle aerodynamic drag
	volatile float drivecycle_drag_1		= 0.0f;
	//Drivecycle Rolling Resistance
	volatile float drivecycle_resistance_1 	= 0.0f;
	//Drivecycle Speed Variable
	volatile float drivecycle_omega_2;
	//Drivecycle Speed Variable
	volatile float omega_err_km1_1;
	//Drivecycle omega_dot_slew
	volatile float drivecycle_omega_dot_slew_2 = 0.001f;
	//Drivecycle omega_dot_km1
	volatile float drivecycle_omega_dot_km1_2 = 0.0f;
	//Drivecycle omega_dot_max
	const float drivecycle_omega_dot_max_2 = 200.0f;
	//Drivecycle omega_dot_min
	const float drivecycle_omega_dot_min_2 = -200.0f;
	
	
	
	// Squared law feedback for fan type load
	volatile float omega_err_sqared_1 		=0.0000f; 
	// Motion controller damping (N/rad/s)
	const float ba_1						=0.0061302599722150f;//0.003537126016096f;//0.0153f; 				
	// Motion controller stiffness (N/rad)
	const float ksa_1						=0.209468422261168f;//0.023649684812170f;//0.45257f;//0.18257f;//			
	// Motion controller integral stiffnes (N/(rad*s))
	const float kisa_1						=2.156900260542138;//0.013469743164952f;//2.156900260542138;//1.080144154422577f;//4.505;//1.89548f;// 				
	// Rated machine torque (Nm)
	const    float t_rated_1				=0.2754f;					
	// Maximum allowable machine torque (Nm)
	const	 float t_max_1					=t_rated_1;  		


// Machine 2 Motion Controller
	// Omega Ref Selection
	volatile float omega_star_select_2		=0.0f;						
	// Desired machine shaft speed (rad/s)
	volatile float omega_star_kp1_2			=0.0f;					
	// Desired machine shaft speed (rad/s)
	volatile float omega_star_2				=0.0f;	
	// Desired machine constant reference
	volatile float omega_star_const_2		=0.0f;
	// Desired machine shaft reference for sine trajectory
	volatile float omega_star_sin_amp_2		=0.0f;
	// Desired machine shaft reference frequency for sine trajectory
	volatile float omega_star_sin_freq_2	=0.0f;
	// Desired machine shaft reference offset for sine trajectory
	volatile float omega_star_sin_offset_2	=0.0f;/*
	// Desired machine shaft reference for square trajectory
	volatile float omega_star_sq_amp_2		=0.0f;
	// Desired machine shaft reference frequency for square trajectory
	volatile float omega_star_sq_freq_2		=0.0f;
	// Desired machine shaft reference offset for square trajectory
	volatile float omega_star_sq_offset_2	=0.0f;*/
		
	//Drivecycle Array Counter
	volatile int drivecycle_array_counter_2 = 0;
	//Drive cycle counter
	volatile int drivecycle_counter_2 		= 0;
	//Drive cycle slew rate
	volatile float drivecycle_slew_2 		= 0;
	//Drive cycle slew for next interval
	volatile float drivecycle_slew_p1_2		= 0;
	//Drive cycle intermediate variable for debugging
	volatile float drivecycle_speed_2 		= 0;
	//Drive cycle acceleration variable
	volatile float drivecycle_omega_dot_2	= 0;
	//Drivecycle torque slew variable
	volatile float tem_star_slew_1 			= 0.01f;
	//Drivecycle loading previous torque variable
	volatile float tem_star_km1_1			= 0;
	
	
	// Velocity error
	volatile float omega_err_kp1_2			=0.0f;					
	// Error accumulator for Ksa
	volatile float omega_err_kp1_accum1_2	=0.0f; 					
	// Second error accumulator for Kisa
	volatile float omega_err_kp1_accum2_2	=0.0f;					
	// Troque command feedforward value (Nm)
	volatile float t_cff_sfb_2				=0.0f;					
	// SFb motion controller loop time
	volatile    float t_sfb_2					=delta_theta_mult/pwm_frequency;	
	
	
	// Motion controller damping (N/rad/s)   USE 2.5 = ba for line fed IM emulation and 0 for others
	const float ba_2						=0.0061302599722150f;//0.00613f;//0.0071f;				//5 Hz = 0.76, 25 Hz = 3.775, 50 Hz = 7.486	
	// Motion controller stiffness (N/rad)
	const float ksa_2					    =0.209468422261168f;//0.0000383f;//			//0.5 Hz = 2.215, 2.5 Hz = 54.999, 5 Hz = 218.102	
	// Motion controller integral stiffnes (N/(rad*s))
	const float kisa_2					    =1.080144154422577f;//0.0445f;//0.0045055f;//0;//0.0045f;//		//0.01 Hz = 0.136, 0.05 Hz = 16.907, 0.1 Hz = 134.091	
	
	
	//Gains for 1500Khz
		// Motion controller damping (N/rad/s)   USE 2.5 = ba for line fed IM emulation and 0 for others
	//volatile float ba_2						=0.76f;				//5 Hz = 0.76, 25 Hz = 3.775, 50 Hz = 7.486	
	// Motion controller stiffness (N/rad)
	//volatile float ksa_2					=2.38f;			//0.5 Hz = 2.215, 2.5 Hz = 54.999, 5 Hz = 218.102	
	// Motion controller integral stiffnes (N/(rad*s))
	//volatile float kisa_2					=0.67f;			//0.01 Hz = 0.136, 0.05 Hz = 16.907, 0.1 Hz = 134.091	
	
	
	
	// Rated machine torque (Nm)
	const    float t_rated_2				=0.2754f;//*0.5f;					
	// Maximum allowable machine torque (Nm)
	const	 float t_max_2				 	=t_rated_2;  		
	// Torque command
	volatile float tem_star_2				=0.0f;	
	// Speed slew rate for motion commands
	volatile float omega_slew_2				=1.0f;

	
	
/* Incremental Implementation Vars */
//Machine 1: Load Incremental Motion Controller
	volatile float ba1 = ba_1;
	volatile float ksa1 = ksa_1;
	volatile float kisa1 = kisa_1;
	volatile float delta_theta_star_1;
	volatile float delta_theta_star_err_1;
	volatile float delta_theta_star_err_accum_1;
	volatile float delta_theta_star_err_accum2_1;
//Machine 2: Drive Incremental Motion Controller
	volatile float ba2 = ba_2;
	volatile float ksa2 = ksa_2;
	volatile float kisa2 = kisa_2;
	volatile float delta_theta_star_2;
	volatile float delta_theta_star_err_2;
	volatile float delta_theta_star_err_accum_2;
	volatile float delta_theta_star_err_accum2_2;
//Incremental Motion Observer
	// Estimated test stand shaft inertia (Kg*m^2)
	
	volatile float j_tot_hat				=0.000019f;//=0.066f;//=0.044f; //=0.022f;				
	volatile float j_tot_hat_inv_1 			=1.0f/j_tot_hat;
//	volatile float bo1 						=5.4f;//= 14.8f;
//	volatile float kso1 					=150.f;//= 508.2f;
//	volatile float kiso1 					=946.f;//= 2640.f;
	volatile float delta_theta_err_1;
	volatile float delta_theta_err_accum_1;
	volatile float delta_theta_err_accum2_1;
	volatile float torq_obsr_ctrl_1;
	volatile float delta_theta_hat_k_1;
	volatile float delta_theta_hat_kp1_1;
	volatile float omega_hat_k_1;
	volatile float omega_hat_kp1_1;
	volatile float omega_dot_hat_k_1;
	volatile float omega_dot_hat_kp1_1;
	
//Incremental Shaft Torque Estimator
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_drive_hat				=0.000019f;//=0.066f;//=0.044f; //=0.022f;				
	volatile float j_drive_hat_inv_2 		=1.0f/j_drive_hat;
	/*volatile float bo_2 					=bo_1;//7.1f;//= 14.8f;
	volatile float kso_2 					=kso_1;//250.f;//= 508.2f;
	volatile float kiso_2 					=212630000.0f;//1300.f;//= 2640.f;*/
	volatile float delta_theta_err_2		=0.0f;
	volatile float delta_theta_err_accum_2	=0.0f;
	volatile float delta_theta_err_accum2_2	=0.0f;
	volatile float torq_obsr_ctrl_2			=0.0f;
	volatile float delta_theta_hat_k_2		=0.0f;
	volatile float delta_theta_hat_kp1_2	=0.0f;
	volatile float omega_hat_k_2			=0.0f;
	volatile float omega_hat_kp1_2			=0.0f;
	volatile float omega_dot_hat_k_2		=0.0f;
	volatile float omega_dot_hat_kp1_2		=0.0f;

// Noise Signal vars
#ifdef _NOISE
	float noise_q;
	float noise_d;
	float noise_amp = 0.5f;
	int flag_start_noise = 0;
	const float bit_32 = 1.0f/2147483648;
#endif

/* Chirp Signal Vars */
#ifdef _CHIRP	
	float f_0             = 100.0f;                   // Chirp starting frequency
	float f_1             = 10000.0f;                 // Chirp terminal frequency (needs to be half as big)
//	float chirp_time	  = 1.0f;//30.f;		   // Duration of chirp in sec
	float chirp_amp		  = 1.0f;
	//float chirp_offset	  = 1.0f; % there shouldn't be any offset for chirp signal
   	
//	float record_time	  = chirp_time + 0.1f;	   // Recording time is 0.1 sec more than chirp time for DC
	float T_c             = 1.0f;      // Chirp Time (in IRQ counts)
	float Df_by_T_c       = (f_1 - f_0)/T_c/2;       // Frequeny difference
	int   k_c             = 0;                     // Chirp counter
	float chirp_q           = 0.0f;                   // Chirp 
	float chirp_d			= 0.0f;
	float posneg			= 1.0f;
	//int chirp_offset_cnt  = 0;
	float flag_start_chirp  = 0;
//	int flag_rec_offset	  = 1;
#endif	

//  dynamic loading 
	float   k_l           = 0;                     // dynamic cycle counter
	volatile int dyn_load_nCycles = 0;			   // number of dynamic loading cycles
	volatile float Cyclical_freq	=0.0f;			//cyclical loading trajectory frequency
	volatile int dyn_load_select_num	=0;		//dynamic load trajectory selection	
	
// dynamic programming
	volatile float Lambda_idx_act;
	volatile int Lambda_idx1;
	volatile int Lambda_idx2;
	volatile float tem_star_dyn_nSample	=0.0f;    // number of switching period within one dynamic load cycle


/***************************************************************/		
/* Field Oriented Control Var Machine 1 */
/*	// Primary Units IM
	// Magnetizing inductance (H)
	const    float lm_hat_1 			=0.0990f;
	// Satator leakage inductance (H)
	const    float lls_hat_1			=0.0052f;			
	// Satator leakage inductance (H)
	const    float llr_hat_1			=0.0052f;			
	// Rotor resistnace (Ohm)
	const    float rr_hat_1 			=0.4000f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_1 			=0.7000f;	*/
	
				

/*	//Secondary
	// Rotor inductance (H)
	const    float lr_hat_1      	 	=lm_hat_1+llr_hat_1;		
	// Rotor inductance (H)	
	const    float ls_hat_1      	 	=lm_hat_1+lls_hat_1;		
	// Stator short cuircuit inductance (H)
	const    float ls_prime_hat_1		=lls_hat_1+llr_hat_1;		
	// rotor time constant (Sec)
	const    float tau_r_hat_1	 	 	=lr_hat_1/rr_hat_1;			
	// inverse of rotor time constant (1/s)
	const    float one_o_tau_r_hat_1 	=1.0f/tau_r_hat_1;  		
	// Rotor time constant discrete time dynamics
	const    float tau_r_num_1			=(1-exp(-t_obsr_1/ tau_r_hat_1));	 
	// Rotor time constant discrete time dynamics
	const    float tau_r_den_1			=exp(-t_sfb_1/ tau_r_hat_1);
	//Electrical reference frame (rad)
	volatile float theta_e_ref_1;
	*/									
	// Maximum d-axis current (A)
	const    float i_d_max_1 			= 5.0f;						
	// Maximum q-axis current (A)
	const    float i_q_max_1 			= 5.0f;					
	// Rotor flux
	volatile float lamda_dr_hat_1		=0.0f;						
	// Calculated machine slip
	volatile float s_omega_e_star_1		=0.0f;						
	// Slip reference frame offset
	volatile float theta_s_star_1		=0.0f;						
	// Desired reference frame offset
	volatile float theta_rf_star_1		=0.0f;						
	// Theta of machine 1
	volatile float theta_r_1=0.0f;
	volatile float iqe_2 = 0.0f;
	volatile float flux_weak;
	volatile float iqe_1 = 0.0f;																												

		// Teknic Inc. SPMSM for Load Machine 1
	// q axis inductance (H)
	const	 float lq_hat_1             =0.000155f;//0.00022;//
	// d axis inductance (H)
	const	 float ld_hat_1             =0.000155f;//0.00022;//
	// Stator resistance (Ohm)
	const	 float rs_hat_1             =0.43665f;
	// Motor Time constant
	const	 float tau_hat_1			=lq_hat_1/rs_hat_1;//0.000488888f;
	// Permanent Magnet Magnetic Flux
	const	 float lambda_hat_1			=0.0067197f;
	
	// Desired torque (Nm)
	volatile float tem_star_1;
	// Desired rotor flux
	volatile float lamda_star_1			=0;//0.50;
	
/***********************************************************/	
	// Current Regulator Vars
	// Propotional current regulator gain
	volatile float k_p_ireg_1 = 0.483574945125336f;//0.188495559215388f;//two_pi*200.0f*lq_hat_1;// 
	//volatile float k_p_ireg_1 = 1.3f; 							
	// Integral Current Regulator gain
	volatile float k_i_ireg_1 =  1371.776432189983f;//548.7105728759933f;//two_pi*200.0f*rs_hat_1;//1371.776432189983f;//0.0f;//36000.0f; 
		// VTP resistance (Ohm)
	const    float r_v_1 = 10.0f; 							
	// VTP resistance (Ohm)
	//const    float r_v_1 = 0.0f; 			    						
	// Synch command current for AIX scope
	volatile float i_q_e_star_1;									
	// Synch command current for AIX scope
	volatile float i_d_e_star_1;									
	// Synch current for AIX scope
	volatile float i_q_e_1;											
	// Synch current for AIX scope
	volatile float i_d_e_1;											
	// dq axis synchronous frame current
	struct      dq i_dqs_e_1;			    						
	// dq axis synchronous frame current err
	struct      dq i_dqs_e_err_1;			    					
	// dq axis synchronous frame current err integral
	struct      dq i_dqs_e_err_int_1;								
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_e_star_1;									
	// dq axis stationary frame commanded voltage
	struct      dq v_dqs_s_star_1;
	// dq axis stationary frame commanded voltage
	struct      dq v_dqs_s_star_foc_1;
	// dq axis stationary frame commanded voltage
	struct      dq v_dqs_s_star_vs_1;	
	struct		dq v_dqs_e_star_vs_1;								
	// dq axis stationary frame commanded current
	struct      dq i_dqs_s_star_1;										
	// dq axis synchronous frame current
	struct      dq i_dqs_e_star_1;									

	/* Field Oriented Control Var Machine 2 */
	//Electrical reference frame (rad)
	volatile float theta_e_ref_2;									
	// Maximum d-axis current (A)
	const    float i_d_max_2 			= 5.0f;					
	// Maximum q-axis current (A)
	const    float i_q_max_2 			= 5.0f;					
	// Rotor flux
	volatile float lamda_dr_hat_2		=0.0f;						
	// Calculated machine slip
	volatile float s_omega_e_star_2		=0.0f;						
	// Slip reference frame offset
	volatile float theta_s_star_2		=0.0f;						
	// Desired reference frame offset
	volatile float theta_rf_star_2		=0.0f;						
	// Theta of machine 2
	volatile float theta_r_2=0.0f;
	volatile float theta_r_comp_2		=0.0f;
	volatile float theta_r_comp_1		=0.0f;
										
	// Desired rotor flux
	volatile float lamda_star_2			=0.0065;//0.50f;						


/* Machine Parameter Estimates 2 */


#ifdef _NON_PARAMETERIZED_MODEL
	// Primary Units
	
/*	SYY's induction machine
// Magnetizing inductance (H)
	const    float lm_hat_2 			=0.099f;				
	// Satator leakage inductance (H)
	const    float lls_hat_2			=0.0052f;					
	// Satator leakage inductance (H)
	const    float llr_hat_2			=0.0052f;					
	// Rotor resistnace (Ohm)
	const    float rr_hat_2 			=0.40f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_2 			=0.70f;	
*/

/*	// Yaskawa's induction machine
	// Magnetizing inductance (H)
	const    float lm_hat_2 			=0.101f;				
	// Satator leakage inductance (H)
	const    float lls_hat_2			=0.0047f;					
	// Satator leakage inductance (H)
	const    float llr_hat_2			=0.006f;					
	// Rotor resistnace (Ohm)
	const    float rr_hat_2 			=0.57f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_2 			=0.70f;	*/

	// Teknic Inc. SPMSM
	// q axis inductance (H)
	const	 float lq_hat_2             =0.000155f;//0.0002202481797808901f;//
	// d axis inductance (H)
	const	 float ld_hat_2             =0.0001531f;//0.0002202481797808901f;//
	// Stator resistance (Ohm)
	const	 float rs_hat_2             =0.43665f;
	// Motor Time constant
	const	 float tau_hat_2			=(lq_hat_2+ld_hat_2)/2/rs_hat_2;//0.0003525180350395053f;
	// Permanent Magnet Magnetic Flux
	const	 float lambda_hat_2			= 0.0067197f;//0.007;//
	// Flux Observer Constant
	const   float c_3_obsr_2			=3.0f*pole_pairs/2.0f;
	// DB-DTFC Quantities
	//volatile float c_1_obsr_2				=0;
	//const	 float c_2_obsr_2				=rs_hat_2*dt/lq_hat_2/lq_hat_2/ld_hat_2/ld_hat_2;
	//volatile float lambda_diff_sq_2			=0;
	//volatile float x1_2						=0;
	//volatile float x2_2						=0;
	//volatile float det_2					=0;

	volatile float dist_d;
	volatile float dist_q;
	
						
	// Observer time step
	volatile 	float t_obsr_2 = delta_theta_mult/pwm_frequency;
	
/*	//IM Parameters:
	//Secondary
	// Rotor inductance (H)
	const   float lr_hat_2      	 		=lm_hat_2+llr_hat_2;																	
	// Rotor inductance (H)
	const   float ls_hat_2      	 		=lm_hat_2+lls_hat_2;																		
	// Coupling coefficient
	const   float sigma_hat_2   	 		=1.f - lm_hat_2*lm_hat_2/(ls_hat_2*lr_hat_2); 											
	// Eqivient resistance
	const   float r_eq_hat_2    	 		=rs_hat_2 + rr_hat_2*lm_hat_2*lm_hat_2/(lr_hat_2*lr_hat_2); 							
	// Short circuit stator time constant
	const   float tau_eq_hat_2  	 		=ls_hat_2*sigma_hat_2/r_eq_hat_2;														
	// rotor time constant (Sec)
	const   float tau_r_hat_2	 		=lr_hat_2/rr_hat_2;																		
	// inverse of rotor time constant (1/s)
	const   float one_o_tau_r_hat_2 		=1.0f/tau_r_hat_2;  																	
	// Rotor time volatileant discrete time dynamics
	const   float tau_r_num_2			=(1-exp(-t_obsr_2/ tau_r_hat_2));														 
	// Rotor time volatileant discrete time dynamics
	const   float tau_r_den_2			=exp(-t_sfb_2/ tau_r_hat_2); 															
	// Stator short circuit inductance (H)
	const   float ls_prime_hat_2			=lls_hat_2+llr_hat_2;																	
	// Flux Observer Constant
	const   float c_1_obsr_2				=  lm_hat_2*(1.0f-tau_r_hat_2/t_obsr_2+tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2));  //lm_hat_2*(1.0f-t_obsr_2/tau_r_hat_2+t_obsr_2/tau_r_hat_2*exp(-t_obsr_2/tau_r_hat_2));
	// Flux Observer Constant
	const   float c_2_obsr_2				=  lm_hat_2*(tau_r_hat_2/t_obsr_2 - tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2)- exp(-t_obsr_2/tau_r_hat_2));  //lm_hat_2*(t_obsr_2/tau_r_hat_2 - t_obsr_2/tau_r_hat_2*exp(-t_obsr_2/tau_r_hat_2)- exp(-t_obsr_2/tau_r_hat_2));  
	// Flux Observer Constant
	const   float c_3_obsr_2				=1.5f*pole_pairs*lm_hat_2/(sigma_hat_2*ls_hat_2*lr_hat_2);
	// Lr / Lm
	const   float lr_hat_2_o_lm_hat_2	= lr_hat_2/lm_hat_2;
	// Ls*sigma 
	const   float ls_hat_2_t_sigma_hat_2 = ls_hat_2*sigma_hat_2;
	// Torque Constant for torque estimate
	const   float c_t_hat_2				= (rs_hat_2*lr_hat_2 + rr_hat_2*ls_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);  
	// Constants for overmodulation prediction
	//const	float c_t_hat_3				= (rr_hat_2*lm_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);
	//const	float c_t_hat_4				= rr_hat_2/(lr_hat_2*sigma_hat_2);
*/
	
#endif





#ifdef _PARAMETERIZED_MODEL

	// Primary Units
	// Magnetizing inductance (H)
	volatile    float lm_hat_2 			=0.099f;
	

/*					
	// Satator leakage inductance (H)
	const    float lls_hat_2			=0.0052f;					
	// Satator leakage inductance (H)
	const    float llr_hat_2			=0.0052f;					
	// Rotor resistnace (Ohm)
	const    float rr_hat_2 			=0.40f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_2 			=0.43665f;						
	// Observer timestep
	const 	 float t_obsr_2 = 1.0f/pwm_frequency;
*/
	//Secondary
	// Stator short cuircuit inductance (H)
	const   float ls_prime_hat_2			=lls_hat_2+llr_hat_2;
	
	// Rotor inductance (H)
	volatile   float lr_hat_2      	 		=lm_hat_2+llr_hat_2;																	
	// Rotor inductance (H)
	volatile   float ls_hat_2      	 		=lm_hat_2+lls_hat_2;																		
	// Coupling coefficient
	volatile   float sigma_hat_2   	 		=1.f - lm_hat_2*lm_hat_2/(ls_hat_2*lr_hat_2); 											
	// Eqivient resistance
	volatile   float r_eq_hat_2    	 		=rs_hat_2 + rr_hat_2*lm_hat_2*lm_hat_2/(lr_hat_2*lr_hat_2); 							
	// Short circuit stator time constant
	volatile   float tau_eq_hat_2  	 		=ls_hat_2*sigma_hat_2/r_eq_hat_2;														
	// rotor time constant (Sec)
	volatile   float tau_r_hat_2	 		=lr_hat_2/rr_hat_2;																		
	// inverse of rotor time constant (1/s)
	volatile   float one_o_tau_r_hat_2 		=1.0f/tau_r_hat_2;  																	
	// Rotor time volatileant discrete time dynamics
	volatile   float tau_r_num_2			=(1-exp(-t_obsr_2/ tau_r_hat_2));														 
	// Rotor time volatileant discrete time dynamics
	volatile   float tau_r_den_2			=exp(-t_sfb_2/ tau_r_hat_2); 															
																		
	// Flux observer constant
	volatile   float c_1_obsr_2				=lm_hat_2*(1.0f-tau_r_hat_2/t_obsr_2+tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2));
	// Flux observer constant
	volatile   float c_2_obsr_2				=lm_hat_2*(tau_r_hat_2/t_obsr_2 - tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2)- exp(-t_obsr_2/tau_r_hat_2));
	// Flux observer constant
//	volatile   float c_3_obsr_2				=1.5f*pole_pairs*lm_hat_2/(sigma_hat_2*ls_hat_2*lr_hat_2);
	// Lr / Lm
	volatile   float lr_hat_2_o_lm_hat_2	= lr_hat_2/lm_hat_2;
	// Ls*sigma
	volatile   float ls_hat_2_t_sigma_hat_2 = ls_hat_2*sigma_hat_2;
	// Torque Constant for torque estimate
	volatile   float c_t_hat_2				= (rs_hat_2*lr_hat_2 + rr_hat_2*ls_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);  
	// Constants for overmodulation prediction
	//const	float c_t_hat_3				= (rr_hat_2*lm_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);
	//const	float c_t_hat_4				= rr_hat_2/(lr_hat_2*sigma_hat_2);
#endif

// Current Regulator Vars
	// Propotional current regulator gain
	volatile float k_p_ireg_2 			= two_pi*500.0f*lq_hat_2;//0.483574945125336f;//4.188501336645156f;//
	//volatile float k_p_ireg_2 			= 1.3f;  					
	// Integral Current Regulator gain
	volatile float k_i_ireg_2 			= two_pi*500.0f*rs_hat_2;//1371.776432189983f;//1288.053;//8600;//
	// Anti-Windup Gain
	volatile float K_anti_windup_2 = 1.0f;			
	// VTP resistance (Ohm)	
	const 	 float r_v_2 				= 1.0f;     	
		// VTP resistance (Ohm)	
	//const 	 float r_v_2 				= 0.0f;   			
	// Synch command current for AIX scope
	volatile float i_q_e_star_2;									
	// Synch command current for AIX scope
	volatile float i_d_e_star_2;									
	// Synch current for AIX scope
	volatile float i_q_e_2;											
	// Synch current for AIX scope
	volatile float i_d_e_2;											
	// dq axis synchronous frame current
	struct      dq i_dqs_e_2;			    						
	// dq axis synchronous frame current err
	struct      dq i_dqs_e_err_2;			    					
	// dq axis synchronous frame current err integral
	struct      dq i_dqs_e_err_int_2;								
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_e_star_2;									
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_s_star_foc_2;
	struct		dq v_dqs_e_star_foc_2;								
	// dq axis voltage command before scaling back 
	struct      dq v_dqs_s_star_pre_2;								
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_s_star_vs_2;
	struct		dq v_dqs_e_star_vs_2;							
	// dq axis stationary frame commanded voltage
	struct      dq v_dqs_s_star_2;									
	// dq axis stationary frame commanded current
	struct      dq i_dqs_s_star_2;										
	// dq axis synchronous frame current
	struct      dq i_dqs_e_star_2;		
	struct 		dq v_dqs_measured_2;							

/* Command Generation */
	// Stepping variable for command generation
	volatile int cmd_gen_step=0;									
	// Torq modulator selection var
	volatile float torq_mod_select = 0.0f;
	// Lowpass Values for Flux Commands
	volatile float torq_mod_select_1 = 0.0f;
	
	volatile int load_select_1 = 1;
	// Lowpass Values for Flux Commands
	volatile float lambda_db_tau = 0.005f;								
	// Slew Rate Limit for Flux Command Changes
	volatile float lambda_db_slew = 0.0005f;							

/* DBCR Vars*/
	// voltage output vector
	struct		dq v_dqs_s_star_dbcr_2;
	struct		dq v_dqs_e_star_dbcr_2;
	struct      dq v_dqs_e_star_dbcr_2_km1;
	// delayed current error
	struct		dq i_dqs_e_err_km1_2;
	struct		dq v_dqs_e_star_dbcr_int_2;
	// constant
	volatile float C_tau_2 		= exp(-dt/tau_hat_2);
	volatile float C_freq_2;
	volatile float speed_switch_2;
	volatile float phi;
	volatile float K_dbcr_2 	= (1.0f-exp(-two_pi*500*dt))*rs_hat_2/(1.0f-C_tau_2);//(1.0f-exp(-two_pi*500*dt))*
	volatile bool  current_obs_low = false;
/* DBDTFC Vars */
	// Delta Torque
	volatile float delta_tem_2;
	// Slope of torque line
	volatile float m_2;
	// Torque line intercept
	volatile float b_2;
	// Volt-s d axis
	volatile float z_d_2;
	// Volt-s q axis
	volatile float z_q_2;
		// Flux command
	volatile float lambda_r_star_2;
	volatile float lambda_r_2;
	volatile float lambda_r_k_q;
	// root calculation
	volatile float root_1_2;
	// root calculation
	volatile float root_2_2;
	// voltage output vector
	struct      dq v_dqs_s_star_db_2;
	struct		dq v_dqs_e_star_db_2;
	struct		dq v_dqs_s_star_db_km1_2;
	// Flux command
	volatile float lambda_s_star_cmd_2 = 0.3f;
	volatile float Lambda_s_DP = 0.3f;
	// Optimal Flux Command
	volatile float lambda_s_star_optimal_2;
	//Optimal Flux Command Limit High
	static float lambda_s_star_limit_high_2 = 0.65f;
	//Optimal Flux Command Limit Low (??)
	static float lambda_s_star_limit_low_2 = 0.2;//0.15f;
	// Flux slection flag
	volatile int lambda_s_star_select_2 = 1;
	// Used for voltage limiting
	volatile float overmod_ratio_2 = 1.0f;
	//used for voltage limiting decisions
	volatile float overmod_angle_2;
	//Used for voltage overmodulation in flux decreasing solution
	volatile float overmod_decrease_2 = 0;
	//Overmodulation addition angle
	volatile float overmod_angle_add_2;
	//Overmodulation angle gain
	const float overmod_angle_gain_2 = 0.1;//0.30;//0.35;//0.425;
	// Angle between commanded voltage and overmodulated voltage vector
	volatile float mu_angle_2 = 0;
	// Recalculated Torque
	volatile float tem_hat_om_kp1_2 = 0;
	//Previous value of torque command
	volatile float tem_star_pre_2 = 0;
	//Torque slew rate
	volatile float tem_star_slew_2 = 0.5;
	//stator flux angle
	volatile float lambda_s_angle;
	//Magnitude of voltage vector
	volatile float v_dqs_s_star_mag_2;
	//Original value of commanded flux linkage
	volatile float lambda_s_star_cmd_pre_2;
	
	
//  5kHz Switching Counter
	volatile int lambda_star_counter_2 = 0;	
	
	
//Trajectory Based Overmodulation Variables
/*
	//flux linkage magnitude
	volatile float lambda_s_ovm_2;
	//Estimated torque
	volatile float tem_hat_ovm_2;	
	//Number of steps to slew
	volatile float steps_ovm;
	//flux linkage slew rate
	volatile float lambda_s_star_ovm_slew_2 =0;;
	//Torque command slew rate
	volatile float tem_star_ovm_slew_2=0;
	//flux linkage command
	volatile float lambda_s_star_ovm_2=0;
	//Torque command
	volatile float tem_star_ovm_2 = 0;
	//Delta torque term
	volatile float delta_tem_ovm_2;
	//Torque wait counter
	volatile float ovm_counter = 0;
	//Overmod flag
	volatile int ovm_flag = 0;
	//Overmodulation slew flag
	volatile int ovm_slew_flag = 0;
	//Torque line intercept before ovm
	volatile float b_2_pre = 0;
*/	
// Stator Current Observer 2 and Flux observers
	// dq axis stationary frame current estimate
	struct      dq i_dqs_s_hat_2;
	struct		dq i_dqs_e_hat_2;				
	// dq axis stationary frame current estimate k+1
	struct      dq i_dqs_s_hat_kp1_2;
	// dq axis rotary frame current estimate k+1
	struct		dq i_dqs_e_hat_kp1_2;			
	// Stator current observer error
	struct      dq i_err_km1_2;
	// Stator current observer error
	struct      dq i_err_2;
	// Stator current observer error accumulator
	struct      dq i_err_accum_2;
	// Current observer correction voltage
	struct		dq v_dqs_current_obsr_2;
	struct      dq v_dqs_current_obsr_harm_decoup_2;
	struct		dq v_dqs_current_obsr_km1_2;
	// Stator current rotor reference frame
	struct		dq i_dqs_r_2;
	// Stator current rotor reference frame
	struct		dq i_dqs_r_km1_2;
	// Rotor Flux rotor reference frame
	struct 		dq lambda_dqr_r_cm_2;
	// Rotor flux stator reference frame
	struct 		dq lambda_dqr_s_cm_2;
	// Rotor flux error stator reference frame
	struct 		dq lambda_dqr_s_err;
	struct		dq lambda_dqr_s_int_km1;
	// Rotor flux error accumulation stator ref frame
	struct 		dq lambda_dqr_s_err_accum;
	struct		dq lambda_dqr_s_int;
	// Rotor flux estimate from voltage model
	struct		dq lambda_dqr_s_vm_2;
	// Stator flux estimate from voltage model
	struct		dq lambda_dqs_s_vm_2;
	// Rotor flux estimate from voltage model
	struct		dq lambda_dqr_s_vm_kp1_2;
	// Stator flux estimate from voltage model
	struct		dq lambda_dqs_s_vm_kp1_2;
	// Stator flux estimate 
	struct 		dq lambda_dqs_s_hat_kp1_2;
	struct		dq lambda_dqs_r_hat_kp1_2;
	struct		dq lambda_dqs_r_hat_k_2;
	// Rotor flux estimate 
	struct 		dq lambda_dqr_s_hat_kp1_2;
	// Stator flux estimate in electrical ref frame
	struct 		dq lambda_dqs_e_2;
		// Stator flux estimate in electrical ref frame in the next time instance
	struct 		dq lambda_dqs_e_2_k1;
	// Rotor flux estimate in electrical ref frame
	struct 		dq lambda_dqr_e_2;
	// Stator Estimated Current Contribution
	struct		dq lambda_qds_s_vm_ff_2;
	// Stator Estimated Current Contribution k-1
	struct		dq lambda_qds_s_vm_ff_km1_2;
	// Stator flux estimate in electrical ref current model
//	struct 		dq lambda_dqs_s_cm_2;
	// Stator flux estimate in electrical ref current model
//	struct 		dq lambda_dqs_s_vm_ol_2;
	// Rotor flux estimate in electrical ref current model
//	struct 		dq lambda_dqr_s_vm_ol_2;

//Fluxes for model predictive control for overmodulation, all in stationary reference frame
/*
	struct	tem_comparison {volatile float best,k0,k1,k2,k3; int v1,v2,v3;} tem;
	struct	dq voltage[50];
	struct	{volatile float q_k0,d_k0,q_k1,d_k1,q_k2,d_k2,q_k3,d_k3;} lambda_s;
	struct	{volatile float q_k0,d_k0,q_k1,d_k1,q_k2,d_k2,q_k3,d_k3;} lambda_r;		
*/

/* Observer gains 10kHz */
	volatile float tem_hat_kp1_2	= 0.0f;
	volatile float tem_hat_kp1_2_k1 = 0.0f;
	volatile float tem_hat_kp1_2_k2 = 0.0f;
	volatile float tem_hat_kp1_2_k3 = 0.0f;
	//Flux
	const float k_1_obsr_2 		= pwm_frequency*(1.0f-exp(-dt*two_pi*(1.0f+100.0f)));//2861.469264146139f;//2666.536808783852f;//950.5622362483201;//624.6393791983395;//317.8890218662933f;//189.4772645898057f;//584.412f;//353.351;//148.447f;
	const float k_2_obsr_2 		= ((2.0f-exp(-dt*two_pi*1.0f)-exp(-dt*two_pi*100.0f))*pwm_frequency-k_1_obsr_2)*pwm_frequency;//3885.863110708599f;//660537.0045389414f;//66427.85073247559f;//28899.29920719680;//3885.863110708599;//1958.190990380970f;//353.6200030129777f;//58160.f;//18400;//3722.0f;
	//Current
	const float k_4_obsr_2 		= rs_hat_2*pwm_frequency*(1.0f-exp(-dt*two_pi*200));// 531.8277582836495f;//1154.826264562097f;//2354.393299072175f;//136.1058716494810;//531.8277582836533f; //270.0905010800872;//
	const float k_3_obsr_2 		= dt*k_4_obsr_2*exp(-dt/tau_hat_2)/(1.0f-exp(-dt/tau_hat_2));// 0.255180189556898f;//0.174497379964347f;//0.378908686771895f;//0.772497214924591f;//0.044657537390012f;//0.174497379964347f; //0.088619076491672f;//
	
	/*  Gains for 1500Hz
	*/
		//Flux
	//volatile float k_1_obsr_2 		= 165.0f;//584.412f;//353.351;//148.447f;
	//volatile float k_2_obsr_2 		= 6930.0f;//58160.f;//18400;//3722.0f;
	//Current
	//volatile float k_3_obsr_2 		= 7.52f; //43.22f;//28.545f;//36.297f;//19.87f; //61.379f;
	//volatile float k_4_obsr_2 		= 2030.0f; //54910.0f;//22030.0f;//37090.0f;//10350.f; //128000.0f;
	//*/
	
	
	static int counter = 0;


/* Efficiency Testing Variables */
/*
	volatile float p_in_1;
	volatile float p_out_1;
	volatile float eff_1;
	volatile float p_in_filt_1=0.0f;
	volatile float p_out_filt_1=0.0f;
	volatile float eff_filt_1=0.0f;
	volatile float p_in_2;
	volatile float p_out_2;
	volatile float eff_2 = 0.0f;
	volatile float p_in_filt_2=0.0f;
	volatile float p_out_filt_2=0.0f;
	volatile float eff_filt_2=0.0f;
	volatile float i_s_mag_2=0.0f;
	volatile float v_s_mag_2=0.0f;
	volatile float i_s_mag_filt_2=0.0f;
	volatile float v_s_mag_filt_2=0.0f;
	volatile float eff_filt_tau = 0.01f;
	volatile float lambda_dqs_e_filt_2_d =0.0f;
	volatile float omega_l_hat_kp1_filt_1 = 0.0f;
	*/
	volatile float p_in_2;

/*  Experimentally Derived Optimal Stator Flux Linkage Commands */
	
	volatile float tem_star_opt_2 = 0;
	/* ME699 old coeff
	static float optimal_flux_coeff_c1_2 = 0.000248517709833f;
	static float optimal_flux_coeff_c2_2 = 0.000004492923521f;
	static float optimal_flux_coeff_c3_2 = -0.006917150695580f;
	static float optimal_flux_coeff_c4_2 = 0.000000286799162f;
	static float optimal_flux_coeff_c5_2 = -0.000179455083220f;
	static float optimal_flux_coeff_c6_2 = 0.093685075375095f;
	static float optimal_flux_coeff_c7_2 = 0.000000010400502f;
	static float optimal_flux_coeff_c8_2 = -0.000002036398953f;
	static float optimal_flux_coeff_c9_2 = -0.000105553574486f;
	static float optimal_flux_coeff_c10_2 = 0.086348778955512f;	

	//new as of 2/1/2012
	static float optimal_flux_coeff_c1_2 = 0.000168509258288f;
	static float optimal_flux_coeff_c2_2 = 0.000016212095876f;
	static float optimal_flux_coeff_c3_2 = -0.007044576914973f;
	static float optimal_flux_coeff_c4_2 = -0.000000367505400f;
	static float optimal_flux_coeff_c5_2 = -0.000152838814544f;
	static float optimal_flux_coeff_c6_2 = 0.095576555822887f;
	static float optimal_flux_coeff_c7_2 = 0.000000016295448f;
	static float optimal_flux_coeff_c8_2 = -0.000001272184341f;
	static float optimal_flux_coeff_c9_2 = -0.000311593850506f;
	static float optimal_flux_coeff_c10_2 = 0.089574393922821f;	
	*/
	
	//Includes Saturation Effects (new as of 6/14/12)
	static float optimal_flux_coeff_c1_2 = 0.000191447131740f;
	static float optimal_flux_coeff_c2_2 = 0.000009070199006f;
	static float optimal_flux_coeff_c3_2 = -0.006486509498936f;
	static float optimal_flux_coeff_c4_2 = -0.000000060625399f;
	static float optimal_flux_coeff_c5_2 = -0.000121742544842f;
	static float optimal_flux_coeff_c6_2 = 0.085012622073901f;
	static float optimal_flux_coeff_c7_2 = -0.000000008490935f;
	static float optimal_flux_coeff_c8_2 = 0.000004759477227f;
	static float optimal_flux_coeff_c9_2 = -0.000819614263429f;
	static float optimal_flux_coeff_c10_2 = 0.120515689946241f;	
	
	
	
	
	volatile float tau_s_prime;
/*
	volatile float tkm1;
	volatile float tkm2;
	volatile float tkm3;
	volatile float tkm4;
	volatile float tkm5;
	volatile float tkm6;
	volatile float tkm7;
	volatile float tkm8;
	volatile float tkm9;
	volatile float tkm10;
	volatile float tkm11;
	volatile float tkm12;
	volatile float tkm13;
	volatile float tkm14;
	volatile float tkm15;
	volatile float tkm16;
	volatile float tkm17;
	volatile float tkm18;
	volatile float tkm19;
	volatile float tkm20;
	volatile float tkm21;
	volatile float tkm22;
	volatile float tkm23;
	volatile float tkm24;
	volatile float tkm25;
	volatile float tkm26;
	volatile float tkm27;
	volatile float tkm28;
	volatile float tkm29;
	volatile float tkm30;
	volatile float tkm31;
	volatile float tkm32;
	volatile float tkm33;
	volatile float tkm34;
	volatile float tkm35;
	volatile float tkm36;
	volatile float tkm37;
	volatile float tkm38;
	volatile float tkm39;
	volatile float tkm40;
	volatile float tkm41;
	volatile float tkm42;
	volatile float tkm43;
	volatile float tkm44;
	volatile float tkm45;
	volatile float tkm46;
	volatile float tkm47;
	volatile float tkm48;
	volatile float tkm49;
	volatile float tkm50;
	volatile float tkm51;
	volatile float tkm52;
	volatile float tkm53;
	volatile float tkm54;
	volatile float tkm55;
	volatile float tkm56;
	volatile float tkm57;
	volatile float tkm58;
	volatile float tkm59;
	volatile float tkm60;
	volatile float tkm61;
	volatile float tkm62;
	volatile float tkm63;
	volatile float tkm64;
	volatile float tkm65;
	volatile float tkm66;
	volatile float tkm67;
	volatile float tkm68;
	volatile float tkm69;
	volatile float tkm70;
	volatile float tkm71;
	volatile float tkm72;
	volatile float tkm73;
	volatile float tkm74;
	volatile float tkm75;
	volatile float tkm76;
*/
	volatile float tem_star_avg;
	volatile int t_i;
	volatile float t_sum;
	volatile float t_avg[500];
	volatile float t_avg_old[500];
/*  Analytically Derived Optimal Stator Flux Linkage Commands */

	static float optimal_flux_coeff_a1_2 = -0.000066078529131f;
	static float optimal_flux_coeff_a2_2 =  0.001999748580251f;
	static float optimal_flux_coeff_a3_2 = -0.022271406159759f;
	static float optimal_flux_coeff_a4_2 =  0.127451583350095f;
	static float optimal_flux_coeff_a5_2 =  0.216511476472037f;
	 
/*  Lagrange Multiplier Loss Minimization Variables and Coefficients (added by Brian 4/10/2011) additions made 9/17/2011*/ 
	/*
	const	float X1		=	(-lm_hat_2*lr_hat_2*sigma_hat_2 + lm_hat_2*rr_hat_2*dt)/(lr_hat_2*lr_hat_2*ls_hat_2*sigma_hat_2*sigma_hat_2);
	const	float X2		=	(-lm_hat_2*dt)/(lr_hat_2*ls_hat_2*sigma_hat_2);				
	const	float X3		=	(-lm_hat_2*lm_hat_2*rr_hat_2*dt)/(lr_hat_2*lr_hat_2*ls_hat_2*ls_hat_2*sigma_hat_2*sigma_hat_2);
	const	float X4		=	((1/(lr_hat_2*sigma_hat_2)) - (rr_hat_2*dt)/(lr_hat_2*lr_hat_2*sigma_hat_2*sigma_hat_2));
	const	float X5		=	dt/(lr_hat_2*sigma_hat_2);
	const	float X6		=	(lm_hat_2*rr_hat_2*dt)/(lr_hat_2*lr_hat_2*ls_hat_2*sigma_hat_2*sigma_hat_2);
	const	float A1		=	1/(ls_hat_2*sigma_hat_2);
	const	float Co1		=	-lm_hat_2/(lr_hat_2*ls_hat_2*sigma_hat_2);
	*/
 // Stator flux optimization from Lagrange method in stator ref frame
 /*
	struct		dq lambda_dqs_e_opt_kp1_2;
	
	volatile float A_A;
	volatile float B_A;
	volatile float A_C;
	volatile float B_C;
	
	volatile float Co2;
	volatile float D2;
	volatile float rfe_hat_2;
	volatile float lambda_s_star_Lag_optimal_2;
	volatile float w_e_hat;
	volatile float w_slip;
	const float T_const     =  3*lm_hat_2/(sigma_hat_2*ls_hat_2*lr_hat_2);
	volatile float P_loss;
	*/
	
//	volatile float theta_e_hat;
//	volatile float theta_stator;
//	volatile float theta_rotor1;
//	volatile float theta_rotor2;
//	volatile float theta_e_hat_km1;
	volatile float omega_slip;
//	volatile float omega_slip_2;
//	volatile float omega_slip_3;
//	volatile float lambda_prec;

//excitation code variables (for MTPA?)
/*
	volatile float alpha_A;
	volatile float alpha_C;

	volatile float omega_e_bar_A;
	volatile float omega_e_bar_C;
		
	volatile float omega_slip_A;
	volatile float omega_slip_C;

	volatile float PtA_iron;
	volatile float PtC_iron;
				
	volatile float delta_q_plus;
	volatile float delta_q_minus;
	volatile float PtC_current;
	volatile float PtA_current;
	struct		dq Pt_c;
	struct		dq Pt_a;
	struct		dq lambda_c;
	struct		dq lambda_a;
	volatile float lambda_s_star_mtpa;
	const float N =6;
	volatile float Inc_flux_d;
	
		*/
	struct		dq	i_dqs_es_2;
	struct		dq	i_dqs_es_2_km1;
	struct		dq	lambda_dqs_e_2_kp1;
	volatile float num;
	volatile float den;
	

	/*
	struct		dq	Pt_1;
	struct		dq	Pt_2;
	struct		dq	Pt_3;
	struct		dq	Pt_4;
	struct		dq	Pt_5;
	
	struct		dq	lambda_1;
	struct		dq	lambda_2;
	struct		dq	lambda_3;
	struct		dq	lambda_4;
	struct		dq	lambda_5;
	
	volatile float Pt1_current;
	volatile float Pt2_current;
	volatile float Pt3_current;
	volatile float Pt4_current;
	volatile float Pt5_current;
	volatile long  theta_idx;
	*/
	volatile float p_iron_2;
	volatile float p_cond_2;
	volatile float p_loss_2;
	struct		dq i_dqr_s_2;
	volatile float ke = 0.0006;
	volatile float kh = 0.2062;
/*	float lambda_s_star_optimal_array_2[1001] = {
0.399,0.4,0.401,0.402,0.403,0.404,0.405,0.406,0.407,0.408,0.409,0.41,0.411,0.412,0.413,0.414,0.415,0.416,0.417,0.418,0.4185,0.4195,0.4205,0.4215,0.4225,0.4235,0.4245,0.4255,0.426,0.427,0.428,0.429,0.43,0.431,0.4315,0.4325,0.4335,0.4345,0.4355,0.436,0.437,0.438,0.439,0.4395,0.4405,0.4415,0.4425,0.443,0.444,0.445,0.4455,0.4465,0.4475,0.4485,0.449,0.45,0.451,0.4515,0.4525,0.4535,0.454,0.455,0.4555,0.4565,0.4575,0.458,0.459,0.4595,0.4605,0.4615,0.462,0.463,0.4635,0.4645,0.465,0.466,0.4665,0.4675,0.4685,0.469,0.47,0.4705,0.471,0.472,0.4725,0.4735,0.474,0.475,0.4755,0.4765,0.477,0.4775,0.4785,0.479,0.48,0.4805,0.481,0.482,0.4825,0.483,0.484,0.4845,0.485,0.486,0.4865,0.487,0.488,0.4885,0.489,0.49,0.4905,0.491,0.4915,0.4925,0.493,0.4935,0.494,0.4945,0.4955,0.496,0.4965,0.497,0.4975,0.498,0.499,0.4995,0.5,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,
0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5005,0.5,0.4995,0.499,0.498,0.4975,0.497,0.4965,0.496,0.4955,0.4945,0.494,0.4935,0.493,0.4925,0.4915,0.491,0.4905,0.49,0.489,0.4885,0.488,0.487,0.4865,0.486,0.485,0.4845,0.484,0.483,0.4825,0.482,0.481,0.4805,0.48,0.479,0.4785,0.4775,0.477,0.4765,0.4755,0.475,0.474,0.4735,0.4725,0.472,0.471,0.4705,0.47,0.469,0.4685,0.4675,0.4665,0.466,0.465,0.4645,0.4635,0.463,0.462,0.4615,0.4605,0.4595,0.459,0.458,0.4575,0.4565,0.4555,0.455,0.454,0.4535,0.4525,0.4515,0.451,0.45,0.449,0.4485,0.4475,0.4465,0.4455,0.445,0.444,0.443,0.4425,0.4415,0.4405,0.4395,0.439,0.438,0.437,0.436,0.4355,0.4345,0.4335,0.4325,0.4315,0.431,0.43,0.429,0.428,0.427,0.426,0.4255,0.4245,0.4235,0.4225,0.4215,0.4205,0.4195,0.4185,0.418,0.417,0.416,0.415,0.414,0.413,0.412,0.411,0.41,0.409,0.408,0.407,0.406,0.405,0.404,0.403,0.402,0.401,0.4,0.399,0.398,0.397,0.396,0.395,0.394,0.393,0.392,0.391,0.39,0.389,0.388,0.387,0.386,0.385,0.384,0.383,0.382,0.381,0.3795,0.3785,0.3775,0.3765,0.3755,0.3745,0.3735,0.3725,0.3715,0.3705,0.369,0.368,0.367,0.366,0.365,0.364,0.363,0.3615,0.3605,0.3595,0.3585,0.3575,0.3565,0.355,0.354,0.353,0.352,0.351,0.3495,0.3485,0.3475,0.3465,0.3455,0.3445,0.343,0.342,0.341,0.34,0.339,0.3375,0.3365,0.3355,0.3345,0.333,0.332,0.331,0.33,0.329,0.3275,0.3265,0.3255,0.3245,0.323,0.322,0.321,0.32,0.3185,0.3175,0.3165,0.3155,0.3145,0.313,0.312,0.311,0.31,0.3085,0.3075,0.3065,0.3055,0.304,0.303,0.302,0.301,0.2995,0.2985,0.2975,0.2965,0.2955,0.294,0.293,0.292,0.291,0.2895,0.2885,0.2875,0.2865,0.285,0.284,0.283,0.282,0.281,0.2795,0.2785,0.2775,0.2765,0.2755,0.274,0.273,0.272,
0.271,0.27,0.2685,0.2675,0.2665,0.2655,0.258,0.2575,0.2575,0.2565,0.256,0.255,0.254,0.253,0.2525,0.2515,0.2505,0.2495,0.249,0.247,0.247,0.246,0.245,0.2445,0.243,0.242,0.2415,0.24,0.239,0.2385,0.237,0.236,0.235,0.234,0.233,0.232,0.231,0.23,0.23,0.2285,0.2275,0.227,0.226,0.2245,0.2245,0.223,0.2225,0.221,0.221,0.219,0.219,0.218,0.217,0.2165,0.2145,0.2145,0.2135,0.213,0.2115,0.211,0.2105,0.2095,0.208,0.208,0.207,0.2065,0.2055,0.205,0.203,0.203,0.2025,0.202,0.201,0.2005,0.1995,0.199,0.198,0.1975,0.197,0.196,0.1955,0.194,0.194,0.1935,0.193,0.1925,0.192,0.1915,0.1905,0.19,0.1895,0.189,0.1885,0.188,0.1875,0.187,0.1865,0.186,0.1855,0.185,0.1845,0.1845,0.184,0.1835,0.183,0.1825,0.1825,0.182,0.182,0.182,0.182,0.181,0.1805,0.18,0.18,0.1795,0.1795,0.1795,0.179,0.179,0.1785,0.1785,0.1785,0.1785,0.178,0.178,0.178,0.178,0.178,0.178,0.178,0.1775,0.1775,0.1775,0.178,0.178,0.178,0.178,0.178,0.178,0.178,0.1785,0.1785,0.1785,0.1785,0.179,0.179,0.1795,0.1795,0.1795,0.18,0.18,0.1805,0.181,0.182,0.182,0.182,0.182,0.1825,0.1825,0.183,0.1835,0.184,0.1845,0.1845,0.185,0.1855,0.186,0.1865,0.187,0.1875,0.188,0.1885,0.189,0.1895,0.19,0.1905,0.1915,0.192,0.1925,0.193,0.1935,0.194,0.194,0.1955,0.196,0.197,0.1975,0.198,0.199,0.1995,0.2005,0.201,0.202,0.2025,0.203,0.203,0.205,0.2055,0.2065,0.207,0.208,0.208,0.2095,0.2105,0.211,0.2115,0.213,0.2135,0.2145,0.2145,0.2165,0.217,0.218,0.219,0.219,0.221,0.221,0.2225,0.223,0.2245,0.2245,0.226,0.227,0.2275,0.2285,0.23,0.23,0.231,0.232,0.233,0.234,0.235,0.236,0.237,0.2385,0.239,0.24,0.2415,0.242,0.243,0.2445,0.245,0.246,0.247,0.247,0.249,0.2495,0.2505,0.2515,0.2525,0.253,0.254,0.255,0.256,0.2565,0.2575,0.2575,0.258,0.2655,0.2665,0.2675,0.2685,0.27,0.271,0.272,0.273,0.274,0.2755,0.2765,0.2775,0.2785,0.2795,0.281,0.282,0.283,0.284,0.285,0.2865,0.2875,0.2885,0.2895,0.291,0.292,0.293,0.294,0.2955,0.2965,0.2975,0.2985,0.2995,0.301,0.302,0.303,0.304,0.3055,0.3065,0.3075,0.3085,0.31,0.311,0.312,0.313,0.3145,0.3155,0.3165,0.3175,0.3185,0.32,0.321,0.322,0.323,0.3245,0.3255,0.3265,0.3275,0.329,0.33,0.331,0.332,
0.333,0.3345,0.3355,0.3365,0.3375,0.339,0.34,0.341,0.342,0.343,0.3445,0.3455,0.3465,0.3475,0.3485,0.3495,0.351,0.352,0.353,0.354,0.355,0.3565,0.3575,0.3585,0.3595,0.3605,0.3615,0.363,0.364,0.365,0.366,0.367,0.368,0.369,0.3705,0.3715,0.3725,0.3735,0.3745,0.3755,0.3765,0.3775,0.3785,0.3795,0.381,0.382,0.383,0.384,0.385,0.386,0.387,0.388,0.389,0.39,0.391,0.392,0.393,0.394,0.395,0.396,0.397,0.398,0.399
	};
	*/
//120rad/s  T=4sin(theta) + 5 optimal trajectory

float t_compressor[72] = {
  0.5,0.5,0.45,0.425,0.4,0.375,0.35,0.34,0.325,0.3,0.35,0.4,0.5,0.6,0.8,0.9,1.2,1.4,1.7,2,2.4,2.75,3,3.5,4,4.4,4.7,5,5.1,5.15,5.25,5.25,5.25,5.25,5.2,5.15,5.1,5,4.7,4.5,4.1,3.8,3.5,3.3,3.05,2.9,2.7,2.55,2.45,2.3,2.1,2,1.85,1.75,1.65,1.55,1.45,1.4,1.35,1.3,1.25,1.2,1.05,1,0.95,0.9,0.85,0.75,0.7,0.65,0.6,0.55
};//nonlinear single cycle compressor load (avg = 4.5 Nm)

/*
float Lambda_dp[20] = {
  0.3,0.325,0.35,0.364,0.375,0.375,0.375,0.35,0.35,0.325,0.3,0.25,0.225,0.2,0.175,0.175,0.2,0.225,0.25,0.275
}; // for light load low frequency. T_offset = 0.3pu, T_amp = 0.2pu, wrm = 100rad/s, 0.5Hz
*/

/*
float Lambda_dp[20] = {
  0.3234,0.3018,0.3232,0.3018,0.3232,0.3018,0.3232,0.3017,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3014,0.3016
}; // for light load high frequency. T_offset = 0.3pu, T_amp = 0.2pu, wrm = 100rad/s, 10Hz
*/

/*
float Lambda_dp[20] = {
  0.4,0.4307,0.44,0.46,0.46,0.48,0.46,0.46,0.44,0.42,0.4,0.36,0.34,0.32,0.3,0.3,0.3,0.32,0.34,0.38
}; // 	for heavy load low frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 0.5Hz
*/

/*
float Lambda_dp[20] = {
  0.4,0.4307,0.44,0.46,0.46,0.46,0.46,0.46,0.44,0.42,0.4,0.36,0.34,0.32,0.3,0.3,0.3,0.32,0.34,0.38
}; // 	for heavy load low frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 0.5Hz
*/

/*
float Lambda_dp[20] = {
  0.4,0.425,0.425,0.45,0.45,0.45,0.45,0.45,0.425,0.4,0.375,0.35,0.325,0.3,0.275,0.275,0.2891,0.3,0.325,0.375
}; // 	for heavy load low frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 0.5Hz adjusted_v2
*/

/*
float Lambda_dp[20] = {
  0.4,0.425,0.442,0.45,0.45,0.45,0.45,0.45,0.425,0.4,0.375,0.35,0.325,0.3,0.275,0.275,0.275,0.3,0.325,0.3605
}; // 	for heavy load low frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 0.5Hz adjusted_v3
*/

/*
float Lambda_dp[20] = {
  0.425,0.425,0.425,0.425,0.425,0.425,0.425,0.425,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4153,0.408
};  //for heavy load high frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 10Hz (nonlinear model)
*/

/*
float Lambda_dp[20] = {
  0.425,0.425,0.425,0.425,0.425,0.425,0.425,0.425,0.4214,0.4032,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4218,0.4068
};  //for heavy load high frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 10Hz (linear model)
*/
/*
float Lambda_dp[20] = {
  0.425,0.45,0.475,0.5,0.5,0.5,0.5,0.475,0.45,0.425,0.4,0.375,0.35,0.325,0.3,0.3,0.3,0.325,0.35,0.375
};  //for heavy load low frequency %T_offset = 0.667pu, T_amp = 0.333pu, wrm = 150rad/s, 0.5Hz (linear model)
*/

/*
float Lambda_dp[20] = {
  0.375,0.375,0.375,0.375,0.375,0.375,0.375,0.375,0.3622,0.3521,0.35,0.35,0.35,0.35,0.35,0.35,0.35,0.35,0.3569,0.3575
};  //for heavy load high frequency %T_offset = 6, T_amp = 4, wrm = 150rad/s, 10Hz (nonlinear model)
*/

/*
float Lambda_dp[20] = {
  0.375,0.375,0.375,0.375,0.375,0.375,0.375,0.375,0.3642,0.3549,0.35,0.35,0.35,0.35,0.35,0.35,0.35,0.3656,0.3578,0.375
};  //for heavy load high frequency %T_offset = 6, T_amp = 4, wrm = 150rad/s, 10Hz (linear model)
*/

/*
float Lambda_dp[20] = {
  0.35,0.375,0.4,0.425,0.425,0.425,0.425,0.425,0.4,0.375,0.35,0.3,0.275,0.25,0.225,0.225,0.225,0.25,0.2903,0.325
};  //for heavy load low frequency %T_offset = 6, T_amp = 4, wrm = 150rad/s, 0.5Hz (nonlinear model)
*/

/*
float Lambda_dp[20] = {
  0.375,0.4,0.425,0.425,0.425,0.425,0.425,0.425,0.4,0.4,0.375,0.35,0.325,0.3,0.3,0.3,0.3,0.325,0.325,0.35
};  //for heavy load low frequency %T_offset = 7, T_amp = 3, wrm = 150rad/s, 0.5Hz (nonlinear model)
*/


float Lambda_dp[20] = {
  0.4,0.425,0.425,0.45,0.45,0.45,0.45,0.45,0.425,0.4,0.375,0.35,0.325,0.3,0.3,0.3,0.3,0.325,0.35,0.375
};  //for heavy load low frequency %T_offset = 7, T_amp = 3, wrm = 150rad/s, 0.5Hz (linear model)


volatile float t_profile_error;
volatile float Kp = 0.001;
//volatile float t_compressor_adapt[72] = {  //adapted 
//0.5,0.5,0.45,0.425,0.4,0.375,0.35,0.34,0.325,0.3,0.35,0.4,0.5,0.6,0.8,0.9,1.2,1.4,1.7,2,2.4,2.75,3,3.5,4,4.4,4.7,5,5.1,5.15,5.25,5.25,5.25,5.25,5.2,5.15,5.1,5,4.7,4.5,4.1,3.8,3.5,3.3,3.05,2.9,2.7,2.55,2.45,2.3,2.1,2,1.85,1.75,1.65,1.55,1.45,1.4,1.35,1.3,1.25,1.2,1.05,1,0.95,0.9,0.85,0.75,0.7,0.65,0.6,0.55
//};
	volatile float wc_dt = 0; //10 Hz delta angle for frequency emulation (carrier frequency)
	volatile float theta_c;
	volatile int theta_idx1;
	volatile int theta_idx2;
	volatile float theta_idx_act;
	volatile float m_interp;
 
	/*
	float COST[7];
	//float FLUX[7];
	int index_min;
	int i;
	float COST_min;
//	float COST_km1;
//	float COST_improv;
	float lambda_s_star_mtpa_km1;
	*/
	
	volatile float lambda_s_star_sin_amp_2		=0.0f;
	volatile float lambda_s_star_sin_freq_2		=0.1f;
	volatile float lambda_s_star_sin_offset_2	=0.0f;
	
	volatile float tem_star_sin_amp_1 = 4.0f;//0.0f; ;(changed)
	volatile float tem_star_sin_offset_1 = 6.0f;
	volatile float tem_star_sin_freq_1;
	volatile float tem_crank;
	volatile float tem_crank_hat;
	volatile float tem_star_sfb_total_2;
	volatile float over_mod_flag;
		

/*	Online Search Optimization (const speed) */
/*
	volatile long  theta_idx_kp1;	
	float delta_Pin_min = 0.1; //watt difference 
	float delta_lambda = 0.005;	
	volatile float p_in_opt[100] 	=  {500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500};
	volatile float test; 
	volatile float lambda_s_opt[100]=   {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5 };
	volatile float lambda_s_test[100] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5 };
*/
	
/*  Loss Minimization Variables */
	
/*	volatile float lambda_dqs_mag_2 = 0;
	volatile float lambda_dqr_mag_2 = 0;
	volatile float loss_copper_2 = 0;
	volatile float loss_copper_1_2 = 0;
	volatile float loss_copper_2_2 = 0;
	static float loss_copper_constant_1_2 = 1/ls_hat_2 + lm_hat_2*lm_hat_2/(ls_hat_2*lr_hat_2*sigma_hat_2);
	static float loss_copper_constant_2_2 = lm_hat_2/(ls_hat_2*lr_hat_2*sigma_hat_2);
	static float loss_copper_constant_3_2 = 1/(lr_hat_2*sigma_hat_2);
	static float loss_copper_constant_4_2 = lm_hat_2/(ls_hat_2*lr_hat_2*sigma_hat_2);*/
	
/* Steady State Data Collection Macro*/
/*
	volatile float ss_collect 				= 0.0f;
	volatile int   ss_collect_timer			=0;
	volatile int   ss_collect_flag 			=0;
	volatile int   ss_collect_dump_count    =0;
	volatile float ss_collect_omega_max 	=120.0f;
	volatile float ss_collect_omega_min 	=40.0f;
	volatile float ss_collect_omega_step 	=10.0f;
	volatile float ss_collect_omega_star 	=5.0f;
	volatile float ss_collect_torque_max 	=10.0f;
	volatile float ss_collect_torque_min 	=1.0f;
	volatile float ss_collect_torque_step 	=1.0f;
	volatile float ss_collect_torque_star 	=1.0f;
	volatile float ss_collect_lambda_rated 	=0.5f;
	volatile float ss_collect_lambda_max 	=0.6f;
	volatile float ss_collect_lambda_min 	=0.3f;
	volatile float ss_collect_lambda_step 	=0.05f;
	volatile float ss_collect_lambda_star 	=0.5f;
	volatile float ss_collect_is_max 		=12.0f;
	volatile float ss_collect_vs_max 		=170.0f;
	*/


/* Dynamic Testing Macro  */
	volatile float 	dyn_macro_flag			= 1.0f;
	volatile float 	dyn_macro_cmd			= 0.0f;
	volatile int	dyn_macro_timer			= 0;
	volatile int	dyn_macro_pretrig		= 20000;
	volatile float 	dyn_macro_lambda_star	= 0.35f;
	volatile float 	dyn_macro_omega_star    = 0.0f;
	volatile float  dyn_macro_torque_cmd    = 0.0f;
	volatile float  dyn_macro_torque_cmd_nxt= 0.0f;
	volatile float  dyn_macro_torque_slew	= 0.0001f;
	volatile float  dyn_macro_torque_offset = 0.0f;


/* HEX Overmodulation  */
/*
	volatile float 	alpha;
	volatile float	alpha_e;
	volatile float	theta_v;
	volatile float 	mu;
	volatile float 	v_dqs_s_star_mag;
	*/
/* Test Variables */

	volatile float test_5=0.0f;

	// Space Vector Modulation
	 /* Space Vector Modulation */
#ifdef _SVM
	volatile float v_mag_svm=0.0f;   
	volatile float v_phs_svm=0.0f;  // in rad
	volatile int sector=0;
	volatile float v_a_svm=0.0f;
	volatile float v_b_svm=0.0f;
	volatile float v_c_svm=0.0f;
	volatile float Tsv_a=0.0f;
	volatile float Tsv_b=0.0f;
	volatile float Tsv_7=0.0f;
	volatile float Tsv_0=0.0f;
#endif

/* Exit overmodulation*/
/*
	struct	dq test1;
	struct	dq test2;
	struct	dq test3;	
*/	
/* Constant Voltage Excitation */
	volatile float v_exc_amp_2 = 2.0f;
	volatile float v_exc_freq_2 = 12.0f;
	volatile int   v_exc_count_2 = 0;
	volatile float v_exc_amp_1 = 1.2f;
	volatile float v_exc_freq_1 = 12.0f;
	volatile int   v_exc_count_1 = 0;
/* for debugging purposes */
	volatile int flag_indicator = STOP;
	volatile int test1 = 0;
	//volatile int test2 = 0;
	//volatile int test3 = 0;
	
	

