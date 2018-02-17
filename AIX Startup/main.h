/*************************************************************************/
/*						DBDTFC Control Program - main.h					 */
/*						v6.0											 */
/*						09/13/2011										 */
/*						Zachary Hurst, Brian Bradley, Yukai Wang		 */
/*************************************************************************/
// v1.0 Original Code Development


// Libaries Necessary to make code work
//#include"hardware.h"
#include"init_pwm.h"
#include"ENCOD_A.h"
#include <irq.h>
#include <math.h>
#include <stdlib.h>
#include "XCSDataLink.h"
//#include <terminal.h>
#include <I2C.h>
#include<shared/extract.h>
#include<shared/nop.h>
#include<shared/lib_clip.h>
#include<terminal.h>
#include <AixScope.h>
#include <Drive_Cycle.h>
#include <Tor_coeff_30.h>
#include <Coeff_3072Hz.h>
#include <Coeff_1500Hz.h>
#include <Coeff_1000Hz.h>
#include <Coeff_500Hz.h>
#include <Current_Obsr_Coeff_3072Hz.h>
#include <Current_Obsr_Coeff_1000Hz.h>
#include <Current_Obsr_Coeff_1500Hz.h>
#include <Current_Obsr_Coeff_500Hz.h>
#include <Flux_Obsr_Coeff_3072Hz.h>
#include <Flux_Obsr_Coeff_1000Hz.h>
#include <Flux_Obsr_Coeff_1500Hz.h>
#include <Flux_Obsr_Coeff_500Hz.h>
#include <Braking_Traj.h>

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
	static const float wn_div			= 1.f/2147483648;
	static const float N				=326.0f;


#ifdef	_SW3kHz	
	// switching frequency of the PWM
	const float pwm_frequency = 3072.0f;//1536.f;// actually running at half the frequency
	const float enc_frequency = 3072.0f;//1536.f;	
#endif

#ifdef	_SW1kHz	
	// switching frequency of the PWM
	const float pwm_frequency = 1000.0f;//1536.f;
	const float enc_frequency = 1000.0f;//1536.f;
#endif

const float pwm_freq_real=pwm_frequency; // the real pwm freq
const float enc_freq_real=enc_frequency;
	
	
/* Structures and defined types */
	// Structure for referencing dq quantities
	struct dq {float d,q;};

	// switching period
	const float dt = 1/pwm_freq_real;
	// motion loop frequency (Multiple of PWM)

	//const float enc_frequency = 0.015f;//;0.015f;//0.1f;
	// communications update frequency
	//const float link_frequency = 500.0f;
	const float link_frequency = 400.f;
	// software generated deadtime
	const float pwm_deadtime = 1.1e-6f;

	// PWM compensation
	// Tuned dead time comp value
	volatile float pwm_t_comp = 0.000f; //0.0045 for 1000Hz   // 0.008 for 3kHz
	
	volatile float pwm_t_comp_real=0.000f;
	// Tuned dead time comp value
	volatile float pwm_inv_comp = 2.0f;
	// Holding var for compenation time calc
	volatile float pwm_t_phase_comp_ref = 0.0f;
	// Modulation index dead time compensation value
	volatile float pwm_t_phase_comp = 0.0f;
	// Inverter FWD voltage drop of power silicon
	volatile float v_fwd_drop = 3.0f;

/* Menu Updating */
	// Number of link cycles to decimate before updating menu routine
	const int menu_decimate = 50;
	// Counter var for menu routine
	volatile int menu_count = 0;
	
/* Motor constants */
	// Number of pole pairs in the machines
	const float pole_pairs = 4.f;									

/* Drive States */
	enum{
	    READY,
	    STARTUP,
	    RUN,
	    STOP,
	    INIT
	};

	// Drive flag
	volatile   int drive_flag = STOP;
	// Number of samples to spend in startup state
	volatile   int startup_count = 500;
	// Number of samples to spend in run
	volatile   int run_count = 0;
	// Number of samples to spend in init - zeroing out current sensors
	volatile   int init_count = 833;
	
	volatile   int flag_indicator;
	

/* System Protection */
	// Max  line current (A)
	volatile float max_current 	= 45.0f;
	// Max dc link voltage (V) 
	volatile float max_voltage 	= 400.0f;
	// Maximum shaft speed (rad/s)
	volatile float omega_max 	= 200.0f;
	
	volatile   int	fault_flag	=0;




/* Function Prototypes */
	// Current regulator loop at 1.5 kHz
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
	const    int log_num_samples = 3072;//6144;//2250;//50000;//2250;//1030;	//(Max around )							
	// Total number of signals to capture
	const    int log_num_channels = 39;//2;//30;								
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
		_Pragma( "section( \"seg_SRAM\" )" )
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
		CMD_OMEGA_SOURCE_2				=201,
		CMD_BA_2						=202,
		CMD_KSA_2						=203,
		CMD_KISA_2						=204,
		CMD_OMEGA_REF_2					=210,
		CMD_OMEGA_SINE_OFFSET_2			=211,
		CMD_OMEGA_SINE_AMP_2			=212,
		CMD_OMEGA_SINE_FREQ_2			=213,
		CMD_OMEGA_SLEW_2 				=214,
		CMD_TORQUE_SLEW_2				=215,
		CMD_TORQUE_MOD_SELECT_2			=240,
		CMD_VS_AMP_2					=251,
		CMD_VS_FREQ_2					=252,
		CMD_FOC_REG_P_2					=260,
		CMD_FOC_REG_I_2					=261,
		CMD_FOC_FLUX_STAR_2				=262,
		CMD_DB_FLUX_SELECT_2			=270,
		CMD_DB_FLUX_STAR_2				=271,
		CMD_DB_FLUX_TAU_2 				=272,
		CMD_DB_FLUX_SLEW_2 				=273,
		CMD_INV_COMP_REF_T_2			=280,
		CMD_INV_COMP_2					=281,
		CMD_INV_V_FWD_DROP_2			=282,
		CMD_FOC_FLUX_SELECT_2           =283,
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
		CMD_K1_2						=400,
		CMD_K2_2						=401,
		CMD_K3_2						=402,
		CMD_K4_2                        =403,
		CMD_TORQUE_2					=500,
		CMD_DBDTFC_MOD					=501,
		CMD_SVM_QS						=502,
		CMD_SVM_DS						=503,
		CMD_FLUX_DID					=504,
		CMD_TORQUE_1					=505,
		CMD_TRI_INJ						=506,
		CMD_ML_MODE						=507,	
		CMD_SCALE_A						=601,
		CMD_SCALE_B						=602,
		CMD_SCALE_C						=603,
		CMD_PHS_B_COMP					=604,
		CMD_VOLT_TEST					=605,
		CMD_MC_INT_TEST					=606, 
		CMD_MAX_TORQ					=607,
		
		CMD_BT_C1						=701,
		CMD_BT_C2						=702,
		CMD_BT_C3						=703,
		CMD_BT_F1						=704,
		CMD_BT_F2						=705,
		CMD_BT_F3						=706,
		CMD_TRI_FREQ					=707,
		CMD_CHIRP_TRI					=800,
		CMD_CHIRP_AMP					=801,
		CMD_CHIRP_STEP					=802,
		CMD_CHIRP_F0					=803,
		CMD_ROTOR_FLUX_TRAJ_TRIGGER     =904,
		CMD_ROTOR_FLUX_TRAJ_FINAL        =905,
		CMD_ROTOR_FLUX_TRAJ_RESET        =906,
		CMD_ROTOR_FLUX_TRAJ_DELTA       =907,
		CMD_ROTOR_FLUX_TRAJ_GAIN        =908,
		
		CMD_ROTOR_FLUX_DBDTFC_FLAG      =9000,
		CMD_ROTOR_FLUX_DR_CMD           =9001,
		
		CMD_MRAS_TRI					=1000,
		CMD_MRAS_RESET					=1001,
		CMD_MRAS_K1						=1002,
		CMD_MRAS_K2						=1003,
		CMD_MRAS_K3						=1004,
		CMD_MRAS_K4						=1005,
		CMD_Rr_MRAS                     =1006,
		CMD_Lm_MRAS                     =1007,
		CMD_TAU_MRAS                    =1008,
	};





/* Voltage Vars*/
	// PWM Commanded Voltage (V)
	volatile float v_a_1;	
	volatile float v_b_1;	
	volatile float v_c_1;
	volatile float v_a_2;
	volatile float v_b_2;	
	volatile float v_c_2;		
	volatile float v_max_2;	
	volatile float v_min_2;	
	volatile float v_n_2;
	// DC Link Voltages
	volatile float v_dc_link;	
	volatile float v_dc_link_filtered;
	
#ifdef _VOLT_SENSOR
	volatile float voltage_EVU;
	volatile float voltage_NEVU;
	volatile float voltage_EWU;
	volatile float voltage_NEWU;
	volatile float voltage_VU;
	volatile float voltage_WU;
	volatile float voltage_VW;
	volatile int volt_cnt;
	volatile int volt_last_cnt=0;
	volatile int delta_volt_cnt;
#endif
	
	// Multilevel inverter dc bus
	volatile float v_dc_u;// u_phase dc bus
	volatile float v_dc_v;//
	volatile float v_dc_w;//
	volatile float v_dc_u_filtered;
	volatile float v_dc_v_filtered;
	volatile float v_dc_w_filtered;
	
	// Modulation index
	volatile float v_a_pwm_1;	
	volatile float v_b_pwm_1;	
	volatile float v_c_pwm_1;	
	volatile float v_a_pwm_2;		
	volatile float v_b_pwm_2;
	volatile float v_c_pwm_2;
	
	
		// Modulation index for multilevel inverter
	volatile float v_u_pwm_1;	
	volatile float v_u_pwm_2;	
	volatile float v_v_pwm_1;	
	volatile float v_v_pwm_2;			
	volatile float v_w_pwm_1;	
	volatile float v_w_pwm_2;	

/* Current Vars */
	// Line current (A)
	volatile float i_a_1;
	volatile float i_b_1;	
	volatile float i_c_1;
	volatile float i_a_2;// used for feedback			
	volatile float i_b_2;										
	volatile float i_c_2;
	volatile float i_a_2_alt;// alternative current sample for feedback			
	volatile float i_b_2_alt;										
	volatile float i_c_2_alt;
	volatile float i_a_2_km1;
	volatile float i_b_2_km1;
	volatile float i_c_2_km1;
	volatile float i_a_2_k;
	volatile float i_b_2_k;
	volatile float i_c_2_k;
										
	volatile float i_a_inv_2;								
	volatile float i_b_inv_2;								
	volatile float i_c_inv_2;
	volatile float i_dc;
	volatile float i_dc_filtered;
	volatile float power_dc;									
	volatile float power_dc_filtered;
	volatile float power_dc_old;
	// dq axis statonary frame current (A)
	struct      dq i_dqs_s_1;
	struct      dq i_dqs_s_2;

	volatile float i_a_scaling=-4.27f;
	volatile float i_b_scaling=-3.291f;
	volatile float i_c_scaling=-3.291f;
	
/* Motion  Vars */
	// Total encoder counts per rev (4x Quad)
	const 	 int   total_enc_cnt			= 3600; 
	// Current encoder count value
	volatile int   enc_cnt 					= 0;
	volatile int   enc_cnt_b 					= 0;
	// Number of Encoder counts since last interrupt 					
	volatile int   delta_cnt 				= 0;
	// Delta count for omega bar calc					
	volatile int   delta_cnt_w_r;	
	// Pervious encoder count value								
	volatile int   last_cnt 				= 0;
	// Flag used for decimating omega bar clac					
	volatile int   motion_flag				= 0;
	// Average Speed (rad/s)					
	volatile float w_bar;
	volatile float w_bar_dec;
	
	// Shaft position (rad)											
	volatile float theta;
		// Shaft position (rad)											
	volatile float theta_test;
	// (k-1)Shaft position (rad)											
	volatile float theta_km1;
	// Theta difference for encoder calc 	
	volatile float delta_theta;
	// RPM of shaft for display  									
	volatile float rpm;												

// Machine 1 Motion Observer
	// Position error (rad)
	volatile float theta_err_2				=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_2		=0.0f;					
	// Torque command feedforward (Nm)
	volatile float t_cff_obsr_2				=0.0f;
	// Position error (rad)
	volatile float theta_err_km1_2				=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_km1_2		=0.0f;	

	
#ifdef _SW3kHz
					
	// Observer damping 
	volatile float bo_2						=29.6f;	//29.6f;represents 10Hz @ 3072Hz Switch frequency operation;
														
	// Observer stiffness
	volatile float kso_2				     =100.0f;//3000.0f;//100.0f;represents 1Hz @ 3072Hz Switch frequency operation;
														// Observer integral stiffness	
	volatile float kiso_2		 			=100.0f;//10000	//105.0f;represents 0.1Hz @ 3072Hz Switch frequency operation;
	
#endif	

#ifdef _SW1kHz
					
	// Observer damping 
	volatile float bo_2						=8.0f;	
															
	// Observer stiffness
	volatile float kso_2				     =240.0f;
													
	// Observer integral stiffness	
	volatile float kiso_2		 			=1000.0f;
#endif														
				
	// Estimated theta position for next loop
	volatile float theta_hat_kp1_2			=0.0f;					
	// Estimated theta position for current count
	volatile float theta_hat_k_2			=0.0f;					
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_p_hat_2				=0.053f;					
	// Inverse of estimated shaft inertia
	volatile float j_p_hat_inv_2			=1.0f/j_p_hat_2;		
	// Observer sample time
	const    float t_obsr_2					=1.0f/pwm_freq_real;
		// Observer sample time
	const    float t_obsr_1					=1.0f/pwm_freq_real;	
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
	
	volatile float omega_encoder			=0.0f;
	volatile float omega_star_stored=0.0f;

	
	
	volatile float omega_star_test_kp1		=0.0f;
	volatile float omega_star_test			=0.0f;
/*	
// Machine 1 Shaft Torque Observer
	// Position error (rad)
	volatile float theta_err_2				=0.0f;					
	// Accumlation of position error
	volatile float theta_err_accum_2		=0.0f;					
	// Torque command feedforward (Nm)
	volatile floaet t_cff_obsr_2			=0.0f;					
	// Observer damping 
	volatile float bo_2						=14.8f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;					
	// Observer stiffness
	volatile float kso_2					=508.2f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;				
	// Observer integral stiffness
	volatile float kiso_2					=2640.0f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;	
	*/
	/* Gains for running in 1500hz	*/
	// Observer damping 
	//	volatile float bo_2						=13.5f;//=10.7f;//=7.1f;//=3.5f;			//=7.0f;//=14.8f;					
	// Observer stiffness
	//	volatile float kso_2					=17349.0f;//=324.0f;//=216.0f;//=107.0f;		//=260.0f;//=508.2f;				
	// Observer integral stiffness
	//	volatile float kiso_2					=42568.0f;//=1631.0f;//=1090.0f;//=540.0f;		//=1340.0f;//=2640.0f;	
	
/*				
	// Estimated theta position for next loop
	volatile float theta_hat_kp1_2			=0.0f;					
	// Estimated theta position for current count
	volatile float theta_hat_k_2			=0.0f;					
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_p_hat_2				=0.022f;//=0.066f;//=0.044f; //=0.022f;				
	// Inverse of estimated shaft inertia
	volatile float j_p_hat_inv_2			=1.0f/j_p_hat_1;			
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


// Machine 2 Heterodyning position tracking observer
struct      dq theta_qd_hat; // feedback signal
struct      dq theta_qd;	// from encoder
struct		dq theta_qd_dec; // decoupling\


volatile float  theta_het_err=0.0f;// feedback error from cross product
volatile float   t_cff_obsr_het=0.0f;// command feedforward;
volatile float   theta_err_accum_het=0.0f;
volatile float   torq_obsr_het=0.0f;

volatile float  omega_l_hat_kp1_het=0.0f;
volatile float   omega_l_hat_het=0.0f;
volatile float   omega_r_hat_kp1_het=0.0f;
volatile float   omega_r_hat_het=0.0f;

volatile float  theta_hat_het_kp1=0.0f;
volatile float   omega_bar_hat_kp1_het=0.0f;
volatile float   theta_hat_het=0.0f;
volatile float   theta_err_accum_het_km1=0.0f;
volatile float   theta_het_err_km1=0.0f;

//GAIN
volatile float bo_het=29.9f;
volatile float kso_het=204.0f;
volatile float kiso_het=210.0f;

	
		
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
	const    float t_sfb_1					=1.0f/pwm_freq_real;
	
	
	#ifdef _SW3kHz

	//[7.22,90.8,102.8] = [20,2,0.2]Hz BW
	volatile float ba_1						=14.22f;											
	// Motion controller stiffness (N/rad)
	volatile float ksa_1					=90.8f;
	// Motion controller integral stiffnes (N/(rad*s))
	volatile float kisa_1					=102.8f;
	#endif
	
	#ifdef _SW1kHz
	//[6.9,86.65,98.15] = [20,2,0.2]Hz BW
	volatile float ba_1						=6.9f;	
	// Motion controller stiffness (N/rad)
	volatile float ksa_1					=86.65f;	
	// Motion controller integral stiffnes (N/(rad*s))
	volatile float kisa_1					=98.15f;
	#endif
		
			
	// Active Inertia
	volatile float ja_1						=0.0f;					
	
	
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
	// Motion controller stiffness (N/rad)
				
	// Rated machine torque (Nm)
	const    float t_rated_1				=41.3f;					
	// Maximum allowable machine torque (Nm)
	volatile	 float t_max_1					=2.0*t_rated_1;  		


// Machine 2 Motion Controller
	// Omega Ref Selection
	volatile float omega_star_select_2		=0;						
	// Desired machine shaft speed (rad/s)
	volatile float omega_star_kp1_2			=0.0f;					
	// Desired machine shaft speed (rad/s)
	volatile float omega_star_2				=0.0f;	
	// Desired machine constant reference
	volatile float omega_star_const_2		=0.0f;
	// Desired machine shaft reference for sine trajectory
	volatile float omega_star_sin_amp_2		=5.0f;
	// Desired machine shaft reference frequency for sine trajectory
	volatile float omega_star_sin_freq_2	=1.0f;
	// Desired machine shaft reference offset for sine trajectory
	volatile float omega_star_sin_offset_2	=30.0f;
	
	volatile float omega_star_km1_2_filtered=0.0f;
	volatile float omega_star_2_filtered=0.0f;
	volatile float omega_star_kp1_2_filtered=0.0f;
	volatile float omega_star_km1_2_unfiltered=0.0f;
	volatile float omega_star_2_unfiltered=0.0f;
	volatile float omega_star_kp1_2_unfiltered=0.0f;
	volatile int ki_hold_flag=0;
		
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
	const    float t_sfb_2					=1.0f/pwm_freq_real;	
	
	
	// triangular waveform reference
	volatile int flag_tri_ref				=0;
	volatile float T_tri					=0.0f;
	volatile float freq_tri					=1.0f;
	volatile float d_omega					=0.0f;
	volatile float timer_tri				=0.0f;
	volatile int flag_tri_down=1;
	
	
	#ifdef _SW3kHz
	/* Not even necessary when use machine 2 as a Torque Drive */
	// Motion controller damping (N/rad/s)   USE 2.5 = ba for line fed IM emulation and 0 for others
	volatile float ba_2						=29.95f;//14.95f;//14.95f; represents 30Hz @ 1536Hz Switch frequency operation;
												  //7.94f; represents 5Hz @ 500Hz Switch frequency operation;	
	// Motion controller stiffness (N/rad)
	volatile float ksa_2					=100.9f;//100.9f;//355.0f;represents 2Hz @ 1536Hz Switch frequency operation;
												   //49.9f; represents 5Hz @ 500Hz Switch frequency operation;	
	// Motion controller integral stiffnes (N/(rad*s))
	volatile float kisa_2					=100.6f;//100.6f;//400.0f;represents 0.2Hz @ 1536Hz Switch frequency operation;
												   //50.6f; represents 5Hz @ 500Hz Switch frequency operation;
												   
	#endif
	
	
	
	#ifdef _SW1kHz
	/* Not even necessary when use machine 2 as a Torque Drive */
	// Motion controller damping (N/rad/s)   USE 2.5 = ba for line fed IM emulation and 0 for others
	volatile float ba_2						=5.5f;//14.95f; represents 30Hz @ 1536Hz Switch frequency operation;
												  //7.94f; represents 5Hz @ 500Hz Switch frequency operation;	
	// Motion controller stiffness (N/rad)
	volatile float ksa_2					=76.65f;//355.0f;represents 2Hz @ 1536Hz Switch frequency operation;
												   //49.9f; represents 5Hz @ 500Hz Switch frequency operation;	
	// Motion controller integral stiffnes (N/(rad*s))
	volatile float kisa_2					=76.15f;//400.0f;represents 0.2Hz @ 1536Hz Switch frequency operation;
												   //50.6f; represents 5Hz @ 500Hz Switch frequency operation;
	#endif

	// Rated machine torque (Nm)
	const    float t_rated_2				=41.3f;					
	// Maximum allowable machine torque (Nm)
	volatile	 float t_max_2				 	=2.0*t_rated_2;  		
	// Torque command
	volatile float tem_star_2				=0.0f;	
	// Speed slew rate for motion commands
	
	#ifdef _SW3kHz
	volatile float omega_slew_2				=0.05f;  //use 0.015 for 1536Hz switching frequency operation
													//use 0.01 for 500Hz and 1000Hz switching frequency operation

	#endif
	
	#ifdef _SW1kHz
	volatile float omega_slew_2				=0.030f;  //use 0.015 for 1536Hz switching frequency operation
													//use 0.01 for 500Hz and 1000Hz switching frequency operation

	#endif
	
	
	
/* Incremental Implementation Vars */
//Machine 1: Load Incremental Motion Controller
//	volatile float ba1 = ba_1;
//	volatile float ksa1 = ksa_1;
//	volatile float kisa1 = kisa_1;
//	volatile float delta_theta_star_1;
//	volatile float delta_theta_star_err_1;
//	volatile float delta_theta_star_err_accum_1;
//	volatile float delta_theta_star_err_accum2_1;
//Machine 2: Drive Incremental Motion Controller
//	volatile float ba2 = ba_2;
//	volatile float ksa2 = ksa_2;
//	volatile float kisa2 = kisa_2;
//	volatile float delta_theta_star_2;
//	volatile float delta_theta_star_err_2;
//	volatile float delta_theta_star_err_accum_2;
//	volatile float delta_theta_star_err_accum2_2;


//Incremental Motion Observer
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_tot_hat				=0.212f;//=0.066f;//=0.044f; //=0.022f;				
	volatile float j_tot_hat_inv_2 			=1.0f/j_tot_hat;
	volatile float bo1 						=57.0f;//= 14.8f;
	volatile float kso1 					=814.f;//= 508.2f;
	volatile float kiso1 					=2009.f;//= 2640.f;
	volatile float delta_theta_err_2=0.0f;
	volatile float delta_theta_err_accum_2=0.0f;
	volatile float delta_theta_err_accum2_2=0.0f;
	volatile float torq_obsr_ctrl_2=0.0f;
	volatile float delta_theta_hat_k_2=0.0f;
	volatile float delta_theta_hat_kp1_2=0.0f;
	volatile float omega_hat_k_2=0.0f;
	volatile float omega_hat_kp1_2=0.0f;
	volatile float omega_dot_hat_k_2=0.0f;
	volatile float omega_dot_hat_kp1_2=0.0f;
	/*
//Incremental Shaft Torque Estimator
	// Estimated test stand shaft inertia (Kg*m^2)
	volatile float j_drive_hat				=0.011f;//=0.066f;//=0.044f; //=0.022f;				
	volatile float j_drive_hat_inv_2 		=1.0f/j_drive_hat;
	volatile float bo2 						=7.1f;//= 14.8f;
	volatile float kso2 					=250.f;//= 508.2f;
	volatile float kiso2 					=1300.f;//= 2640.f;
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
*/

/* Chirp Signal Vars */	
	float f_0             = 10.f;                   // Chirp starting frequency
	float f_1             = 300.f;                 // Chirp terminal frequency (needs to be half as big)
	float chirp_time	  = 1.0f;//30.f;		   // Duration of chirp in sec
	float chirp_amp		  = 1.0f;

	int   T_c             = chirp_time*pwm_freq_real;      // Chirp Time (in IRQ counts)
	float Df_by_T_c       = (f_1 - f_0)/T_c;       // Frequeny difference
	float k_c           = 0;                     // Chirp counter
	float chirp           = 0.f;                   // Chirp 
	//int chirp_offset_cnt  = 0;
	float flag_start_chirp  = 0;
	float flag_statr_chirp_last=0;
//	int flag_rec_offset	  = 1;
	volatile float tem_star_2_bf_chirp=0.0f;

// Chirp time increment/counter
 volatile long x_chirp = 0;
 // Chirp frequency (Hz)
 volatile float f_chirp = 0.0f;
 // Chirp max frequency (Hz)
 volatile float w_chirp = 100.0f*0.5f/8.0f; //Must divide by length of test for max frequency
 volatile float td = 0.0f;	
		
/* Field Oriented Control Var Machine 1 */
	// Primary Units
	// Magnetizing inductance (H)
	const    float lm_hat_1 			=0.0294f;
	// Satator leakage inductance (H)
	const    float lls_hat_1			=0.00207f;			
	// Satator leakage inductance (H)
	const    float llr_hat_1			=0.00246f;			
	// Rotor resistnace (Ohm)
	const    float rr_hat_1 			=0.4010f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_1 			=0.3960f;	
				

	//Secondary
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
	// Maximum d-axis current (A)
	const    float i_d_max_1 			= 20.0f;						
	// Maximum q-axis current (A)
	const    float i_q_max_1 			= 30.0f;					
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
	// Desired torque (Nm)
	volatile float tem_star_1=0;										
	// Desired rotor flux
	volatile float lamda_star_1			=0.30;									

	// Current Regulator Vars\
	
	/* not necessary to change if using complex vector */
	// Propotional current regulator gain
	volatile float k_p_ireg_1 = 1.44f;//1.44f; 	@ 1536Hz Switch frequency operation;
									  //1.27f; represents @ 500Hz Switch frequency operation;						
	// Integral Current Regulator gain
	volatile float k_i_ireg_1 = 525.5f;//525.5f;    @ 1536Hz Switch frequency operation;
									   //80.0f; represents @ 500Hz Switch frequency operation;	
		// VTP resistance (Ohm)
	const    float r_v_1 = 1.49f; 							
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
	// dq axis stationary frame commanded current
	struct      dq i_dqs_s_star_1;										
	// dq axis synchronous frame current
	struct      dq i_dqs_e_star_1;											


		/* Field Oriented Control Var Machine 2 */
	//Electrical reference frame (rad)
	volatile float theta_e_ref_2;									
	// Maximum d-axis current (A)
	const    float i_d_max_2 			= 20.0f;					
	// Maximum q-axis current (A)
	const    float i_q_max_2 			= 30.0f;					
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
	// Desired rotor flux
	volatile float lamda_star_2			=0.30f;		
	volatile float lambda_foc_star_cmd_2=0.30f;				


/* Machine Parameter Estimates 2 */


#ifdef _NON_PARAMETERIZED_MODEL
	// Primary Units
	// Magnetizing inductance (H)
	const  float lm_hat_2 			=0.035f;//0.035f;//0.0294f;					
	// Satator leakage inductance (H)
	const    float lls_hat_2			=0.0021f;					
	// Satator leakage inductance (H)
	const    float llr_hat_2			=0.0025f;					
	// Rotor resistnace (Ohm)
	const    float rr_hat_2 			=0.25f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_2 			=0.3960f;						
	// Observer time step
//	const 	float t_obsr_2 = 1.0f/pwm_freq_real;
	//Secondary
	// Rotor inductance (H)
	const   float lr_hat_2      	 		=lm_hat_2+llr_hat_2;																	
	// Rotor inductance (H)
	const  float ls_hat_2      	 		=lm_hat_2+lls_hat_2;																		
	// Coupling coefficient
	const   float sigma_hat_2   	 		=1.0f - lm_hat_2*lm_hat_2/(ls_hat_2*lr_hat_2); 											
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
	const	float c_t_hat_3				= (rr_hat_2*lm_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);
	const	float c_t_hat_4				= rr_hat_2/(lr_hat_2*sigma_hat_2);
	
	const   float c_exp_RToverL         =exp(-r_eq_hat_2/ls_prime_hat_2/pwm_freq_real);

#endif



#ifdef _PARAMETERIZED_MODEL
	// Primary Units
	// Magnetizing inductance (H)
	volatile    float lm_hat_2 			=0.0294f;					
	// Satator leakage inductance (H)
	const    float lls_hat_2			=0.0021f;					
	// Satator leakage inductance (H)
	const    float llr_hat_2			=0.0025f;					
	// Rotor resistnace (Ohm)
	const    float rr_hat_2 			=0.401f;						
	// Stator resistnace (Ohm)
	const    float rs_hat_2 			=0.3960f;					
	// Observer timestep
//	volatile 	float t_obsr_2 = 1.0f/pwm_freq_real;

	//Secondary
	// Rotor inductance (H)
	volatile   float lr_hat_2      	 		=lm_hat_2+llr_hat_2;																	
	// Rotor inductance (H)
	volatile   float ls_hat_2      	 		=lm_hat_2+lls_hat_2;																		
	// Coupling coefficient
	volatile   float sigma_hat_2   	 		=1.0f - lm_hat_2*lm_hat_2/(ls_hat_2*lr_hat_2); 											
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
	// Stator short cuircuit inductance (H)
	volatile   float ls_prime_hat_2			=ls_hat_2*sigma_hat_2;																	
	// Flux observer constant
	volatile   float c_1_obsr_2				=lm_hat_2*(1.0f-tau_r_hat_2/t_obsr_2+tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2));
	// Flux observer constant
	volatile   float c_2_obsr_2				=lm_hat_2*(tau_r_hat_2/t_obsr_2 - tau_r_hat_2/t_obsr_2*exp(-t_obsr_2/tau_r_hat_2)- exp(-t_obsr_2/tau_r_hat_2));
	// Flux observer constant
	volatile   float c_3_obsr_2				=1.5f*pole_pairs*lm_hat_2/(sigma_hat_2*ls_hat_2*lr_hat_2);
	// Lr / Lm
	volatile   float lr_hat_2_o_lm_hat_2	= lr_hat_2/lm_hat_2;
	// Ls*sigma
	volatile   float ls_hat_2_t_sigma_hat_2 = ls_hat_2*sigma_hat_2;
	// Torque Constant for torque estimate
	volatile   float c_t_hat_2				= (rs_hat_2*lr_hat_2 + rr_hat_2*ls_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);  
	// Constants for overmodulation prediction
	volatile	float c_t_hat_3				= (rr_hat_2*lm_hat_2)/(ls_hat_2*lr_hat_2*sigma_hat_2);
	volatile	float c_t_hat_4				= rr_hat_2/(lr_hat_2*sigma_hat_2);
	
	volatile   float c_exp_RToverL         =exp(-r_eq_hat_2/ls_prime_hat_2/pwm_freq_real);
	#endif

// Current Regulator Vars
	// Propotional current regulator gain
		/* not necessary to change if using complex vector */

	volatile float k_p_ireg_2 			= 2.14f;//2.14f;	represents 20Hz @ 1536Hz Switch frequency operation;
												//1.4f; represents @ 500Hz Switch frequency operation;				
	// Integral Current Regulator gain
	volatile float k_i_ireg_2 			= 150.0f;//150.0f; @ 1536Hz Switch frequency operation;	
												 //40.0f; represents @ 500Hz Switch frequency operation;			
	// VTP resistance (Ohm)	
	const 	 float r_v_2 				= 1.49f;     	
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
	// dq axis voltage command before scaling back 
	struct      dq v_dqs_s_star_pre_2;								
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_s_star_vs_2;								
	// dq axis stationary frame commanded voltage
	struct      dq v_dqs_s_star_2;									
	// dq axis stationary frame commanded current
	struct      dq i_dqs_s_star_2;										
	// dq axis synchronous frame current
	struct      dq i_dqs_e_star_2;
	
	// dq axis synchronous frame commanded voltage
	struct      dq v_dqs_s_star_vs_1;
	
	
	// complex vector voltage output;
	
	struct dq v_dqs_e_star_1_cv;									
	struct dq v_dqs_e_star_1_km1;
	struct dq i_dqs_e_err_1_km1;
	
	#ifdef _SW3kHz
	volatile float k_cv_1=2.5f; 
	volatile float k_cv_2=2.5f;       
	#endif 
	
	#ifdef _SW1kHz
	volatile float k_cv_1=1.80f; 
	volatile float k_cv_2=1.80f;       
	#endif 
	
	volatile float omega_e_1;
	
	
	struct dq v_dqs_e_star_2_cv;									
	struct dq v_dqs_e_star_2_km1;
	struct dq i_dqs_e_err_2_km1;

	
	volatile float omega_e_2;
	
	volatile float Rv_VTP=3*r_eq_hat_2;
	const    float VTP_dgtl=r_eq_hat_2*(exp(-Rv_VTP/ls_prime_hat_2*dt)-1)/(-exp(-r_eq_hat_2/ls_prime_hat_2*dt)+1);
	
/* Command Generation */
	// Stepping variable for command generation
	volatile int cmd_gen_step=0;									
	// Torq modulator selection var
	volatile float torq_mod_select = 1.0f;
	// Lowpass Values for Flux Commands
	volatile float lambda_db_tau = 0.005f;								
	// Slew Rate Limit for Flux Command Changes
	volatile float lambda_db_slew = 0.001f;	
	volatile float lambda_foc_slew=0.002f;						


/* DBDTFC Vars */
	// Delta Torque
	volatile float delta_tem_2;
	// Slope of torque line
	volatile float m_2;
	// Torque line intercept
	volatile float b_2;
	// Volt-s d axis
	volatile float z_d_2;
	volatile float z_d_2_2;
	// Volt-s q axis
	volatile float z_q_2;
	volatile float z_q_2_2;
		// Flux command
	volatile float lambda_s_star_2;

	// root calculation
	volatile float root_1_2;
	// root calculation
	volatile float root_2_2;
	
		volatile float root_2_new;
	
	// voltage output vector
	struct      dq v_dqs_s_star_db_2;
		// voltage output vector
	struct      dq v_dqs_e_star_db_2;
	// Flux command
	volatile float lambda_s_star_cmd_2 = 0.3f;
	// Optimal Flux Command
	volatile float lambda_s_star_optimal_2;
	//Optimal Flux Command Limit High
	static float lambda_s_star_limit_high_2 = 0.6f;
	//Optimal Flux Command Limit Low
	static float lambda_s_star_limit_low_2 = 0.13f;
	// Flux slection flag
	volatile float lambda_s_star_select_2 = 1.0f;
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
	volatile float tem_star_slew_2 = 41.3*0.1;
	//stator flux angle
	volatile float lambda_s_angle;
	//Magnitude of voltage vector
	volatile float v_dqs_s_star_mag_2;
	//Original value of commanded flux linkage
	volatile float lambda_s_star_cmd_pre_2;
	
	
//  5kHz Switching Counter
	volatile int lambda_star_counter_2 = 0;	
	
	
//Trajectory Based Overmodulation Variables

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
	
// Stator Current Observer 2 and Flux observers
	// dq axis stationary frame current estimate
	struct      dq i_dqs_s_hat_2;				
	// dq axis stationary frame current estimate k+1
	struct      dq i_dqs_s_hat_kp1_2;			
	// Stator current observer error
	struct      dq i_err_km1_2;
	// Stator current observer error
	struct      dq i_err_2;
	// Stator current observer error accumulator
	struct      dq i_err_accum_2;
	// Current observer correction voltage
	struct		dq v_dqs_current_obsr_2;
	// Stator current rotor reference frame
	struct		dq i_dqs_r_2;
	// Stator current rotor reference frame
	struct		dq i_dqs_r_km1_2;
	// Rotor Flux rotor reference frame
	struct 		dq lambda_dqr_r_cm_2;
	// Rotor flux stator reference frame
	struct 		dq lambda_dqr_s_cm_2;

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
	// Rotor flux estimate 
	struct 		dq lambda_dqr_s_hat_kp1_2;
	// Stator flux estimate in electrical ref frame
	struct 		dq lambda_dqs_e_2;
		// Stator flux estimate in electrical ref frame in the next time instance
	struct 		dq lambda_dqs_e_2_k1;
	// Rotor flux estimate in electrical ref frame
	struct 		dq lambda_dqr_e_2;
	// Stator flux estimate in electrical ref current model
	struct 		dq lambda_dqs_s_cm_2;
	// Stator flux estimate in electrical ref current model
	struct 		dq lambda_dqs_s_vm_ol_2;
	// Rotor flux estimate in electrical ref current model
	struct 		dq lambda_dqr_s_vm_ol_2;
	// Rotor flux error stator reference frame
	struct 		dq lambda_dqr_s_err;
		// Rotor flux error stator reference frame
	struct 		dq lambda_dqr_s_err_accum;
	
	
/* Flux Observer on a rotor reference frame */
	// Rotor flux error rotor reference frame
	struct 		dq lambda_dqr_r_err;
	// Rotor flux error accumulation rotor ref frame
	struct 		dq lambda_dqr_r_err_accum;
	struct 		dq lambda_dqs_r_vm_2;
	struct 		dq lambda_dqr_r_vm_2;
	struct 		dq lambda_dqs_r_vm_kp1_2;
	struct 		dq lambda_dqr_r_vm_kp1_2;	
	struct 		dq v_dqs_r_star_2;
	volatile float tem_hat_r_vm_kp1_2=0;
	struct 		dq lambda_dqs_s_from_r_vm_2;
	struct 		dq lambda_dqr_s_from_r_vm_2;
	struct 		dq lambda_dqs_r_from_s_vm_2;
	struct 		dq lambda_dqr_r_from_s_vm_2;
	
	struct 		dq i_dqs_r_hat_2;
	struct 		dq i_dqs_r_hat_kp1_2;
	struct 		dq i_err_r_2;
	struct 		dq i_err_accum_r_2;
	struct 		dq v_dqs_current_obsr_r_2;
	struct 		dq lambda_did_r_reduced;
	
		
	volatile float cos_wr_t=0;
	volatile float sin_wr_t=0;
	volatile float sigma_ls_wr=0;
	volatile float one_over_den=0;
	struct 		dq i_dqs_s_from_r_2;
	
/* Improved Flux Observer added by Yukai */	
	// Stator flux estimate 
	struct 		dq lambda_dqs_s_hat_kp1_imp_2;
	// Rotor flux estimate 
	struct 		dq lambda_dqr_s_hat_kp1_imp_2;
	
	struct      dq lambda_dqr_s_did_2;
	
	struct 		dq lambda_dqr_s_imp_2;// calculate from measured current
	struct 		dq lambda_dqs_s_imp_2;// calculate from measured current
	struct 		dq lambda_dqr_s_hat_imp_2; // roll back from advanced value
	struct 		dq lambda_dqs_s_hat_imp_2;// roll back from advanced value
	struct 		dq lambda_dqr_s_hat_imp_err;
	struct 		dq lambda_dqs_s_hat_imp_err;
	struct 		dq lambda_dqr_s_err_imp_accum;
	struct 		dq lambda_dqs_s_err_imp_accum;
	struct 		dq lambda_dqr_cl_imp;
	struct 		dq lambda_dqs_cl_imp;
	
	
	volatile float tem_hat_kp1_imp_2=0.0f;
	volatile float lambda_ds_ras_2=0.0f;
	volatile float lambda_qr_ras_2=0.0f;
	volatile float lambda_dr_ras_2=0.0f;
	volatile float lambda_ds_ras_kp1=0.0f;
	volatile float lambda_qs_ras_kp1=0.0f;
	volatile float lambda_dr_ras_kp1=0.0f;
	volatile float lambda_qr_ras_kp1=0.0f;
	volatile float v_qs_ras_2=0.0f;
	volatile float v_ds_ras_2=0.0f;

	
/* A full order Flux Observer on a rotor reference frame */
	// Rotor flux error rotor reference frame
	struct 		dq lambda_dqr_r_err_full;
	// Rotor flux error accumulation rotor ref frame
	struct 		dq lambda_dqr_r_err_accum_full;
	struct 		dq lambda_dqs_r_vm_full_2;
	struct 		dq lambda_dqr_r_vm_full_2;
	struct 		dq lambda_dqs_r_vm_kp1_full_2;
	struct 		dq lambda_dqr_r_vm_kp1_full_2;
		
	volatile float tem_hat_r_vm_kp1_full_2=0;
	struct 		dq lambda_dqs_s_from_r_vm_full_2;
	struct 		dq lambda_dqr_s_from_r_vm_full_2;
	
	struct 		dq i_dqs_r_vm_full_2;

	struct 		dq i_dqs_r_vm_full_kp1_2;
	struct 		dq i_dqs_r_err_full;
	struct 		dq i_dqs_r_err_accum_full;
	struct 		dq v_dqs_current_obsr_r_full_2;
	struct 		dq lambda_did_r_full;
	
	// Current observer correction voltage
	struct		dq v_dqs_current_obsr_full_2;

//Fluxes for model predictive control for overmodulation, all in stationary reference frame
/*
	struct	tem_comparison {volatile float best,k0,k1,k2,k3; int v1,v2,v3;} tem;
	struct	dq voltage[50];
	struct	{volatile float q_k0,d_k0,q_k1,d_k1,q_k2,d_k2,q_k3,d_k3;} lambda_s;
	struct	{volatile float q_k0,d_k0,q_k1,d_k1,q_k2,d_k2,q_k3,d_k3;} lambda_r;		
*/

/* Observer gains */
	volatile float tem_hat_kp1_2	= 0.0f;
	volatile float tem_hat_current_kp1_2	= 0.0f;
	volatile float tem_hat_kp1_2_k1 = 0.0f;
	volatile float tem_hat_kp1_2_k2 = 0.0f;
	volatile float tem_hat_kp1_2_k3= 0.0f;
	//Flux   (165.8,6941) =(20Hz,10Hz)
	
	
/*************************** gain temperailly used for two-level Inverter *****************************************/	
#ifdef _SW3kHz //(15,2, 70,7)
	volatile float k_1_obsr_2 		= 50.0f;
	volatile float k_2_obsr_2 		= 500.0f; 	
	volatile float k_3_obsr_2 		= 1.5f;							  				
	volatile float k_4_obsr_2 		= 200.1f;													
#endif	
		

#ifdef _SW1kHz
	volatile float k_1_obsr_2 		= 65.0f;
	volatile float k_2_obsr_2 		= 700.0f; 	
	volatile float k_3_obsr_2 		= 1.5f;							  				
	volatile float k_4_obsr_2 		= 100.1f;													
#endif	

// used for rotor reference frame
//k1_qd=k_1_q-j*k_1_d
volatile float k_1_q=34.0;
volatile float k_1_d=0.0;//should be a function of speed
volatile float k_2_q=182.0;
volatile float k_2_d=0.0;//should be zero
volatile float k_3_q=1;
volatile float k_3_d=0.0;//should be a function of speed
volatile float k_4_q=50.0;
volatile float k_4_d=0.0;//should be a function of speed
volatile float slope_k3=0.0041;//should be a function of speed
volatile float slope_k4=-0.0017;//should be a function of speed							
/*************************** END gain temperailly used for two-level Inverter *****************************************/		
/*************************** gain temperailly used for Multilevel Inverter *****************************************/	
//#ifdef _SW3kHz //(15,2, 70,7)
//	volatile float k_1_obsr_2 		= 100.0f;
//	volatile float k_2_obsr_2 		= 1000.0f; 	
//	volatile float k_3_obsr_2 		= 1.35f;							  				
//	volatile float k_4_obsr_2 		= 83.9f;													
//#endif	
		
//#ifdef _SW1kHz
//	volatile float k_1_obsr_2 		= 94.0f;
//	volatile float k_2_obsr_2 		= 1000.0f; 	
//	volatile float k_3_obsr_2 		= 1.15f;							  				
//	volatile float k_4_obsr_2 		= 75.4f;													
//#endif								
/*************************** END gain temperailly used for Multilevel Inverter *****************************************/

			
									  			
	
	volatile float k_5_obsr_2 		=0.07f;  	//0.07.0f; 	@ 3072Hz Switch frequency operation;
									 		 	//0.6f;@ 1kHz Switch frequency operation;
									 		 		
	volatile float k_6_obsr_2 		= 2.01f;    //52.20f; 	@ 3072Hz Switch frequency operation;
												////150.20f; 	@ 1kHz Switch frequency operation;
	volatile float k_7_obsr_2		=0.3f;
	volatile float k_8_obsr_2		=100.0f;;								  			
	
	
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

/*  Experimentally Derived Optimal Stator Flux Linkage Commands */
	
	volatile float tem_star_opt_2 = 0;
	volatile float omega_star_opt_2 = 0;

	/* optimal flux coeff */
	static float optimal_flux_coeff_c1_2 = 0.1326f;
	static float optimal_flux_coeff_c2_2 = 1.175f;
	static float optimal_flux_coeff_c3_2 = -0.06421f;
	static float optimal_flux_coeff_c4_2 = -1.192f;
	static float optimal_flux_coeff_c5_2 = -0.03697f;
	static float optimal_flux_coeff_c6_2 = 0.01321f;
	static float optimal_flux_coeff_c7_2 = 0.3623f;
	static float optimal_flux_coeff_c8_2 = 0.05717f;
	static float optimal_flux_coeff_c9_2 = -0.002499f;
	static float optimal_flux_coeff_c10_2 = -0.01508f;	
	
	static float rated_torque=41.3f;
	static float rated_speed=89.5f;
	static float lambda_db_max=0.6f;
	static float lambda_db_min=0.05f;
/* model based search */	
	volatile float p_iron_2;
	volatile float p_cond_2;
	volatile float p_loss_2;
	volatile float ke = 0.1735;
	volatile float kh = 3.5862;
	volatile float opt_flux_c1;
	volatile float opt_flux_c2;
	volatile float tem_star_2_abs;

	volatile float opt_const_11=2.0/3.0/(pole_pairs)/(pole_pairs)*(rr_hat_2)*(ls_hat_2*ls_hat_2)/lm_hat_2/lm_hat_2;

	volatile float opt_const_12=2.0/3.0/pole_pairs/pole_pairs*(rs_hat_2)*(lr_hat_2*lr_hat_2)*(ls_hat_2*ls_hat_2)/lm_hat_2/lm_hat_2/lm_hat_2/lm_hat_2;	
	volatile float opt_const_13=ke/2.0/pi/pi*4.0/9.0/pole_pairs/pole_pairs*(rr_hat_2*rr_hat_2)*(ls_hat_2*ls_hat_2)/lm_hat_2/lm_hat_2;
	
	volatile float opt_const_21=ke/4.0/pi/pi*pole_pairs*pole_pairs*lm_hat_2*lm_hat_2/(ls_hat_2*ls_hat_2);
	volatile float opt_const_22=kh/2.0/pi*pole_pairs*lm_hat_2*lm_hat_2/(ls_hat_2*ls_hat_2);
	volatile float opt_const_23=3.0/2.0*(rs_hat_2)/(ls_hat_2*ls_hat_2);
	
	
	
	// Optimal Flux Command (model based)
	volatile float lambda_s_star_opt_mb_2;
	volatile float omega_star_mb_2;


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
	//volatile float t_avg[500];
	//volatile float t_avg_old[500];
/*  Analytically Derived Optimal Stator Flux Linkage Commands 

	static float optimal_flux_coeff_a1_2 = -0.000066078529131f;
	static float optimal_flux_coeff_a2_2 =  0.001999748580251f;
	static float optimal_flux_coeff_a3_2 = -0.022271406159759f;
	static float optimal_flux_coeff_a4_2 =  0.127451583350095f;
	static float optimal_flux_coeff_a5_2 =  0.216511476472037f;
	 */
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
	
 // Stator flux optimization from Lagrange method in stator ref frame
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
	
	*/
	
	const float T_const     =  3*lm_hat_2/(sigma_hat_2*ls_hat_2*lr_hat_2);
	volatile float P_loss;
	
//	volatile float theta_e_hat;
	volatile float theta_stator;
	volatile float theta_rotor1;
//	volatile float theta_rotor2;
//	volatile float theta_e_hat_km1;
	volatile float omega_slip;
//	volatile float omega_slip_2;
//	volatile float omega_slip_3;
//	volatile float lambda_prec;

//excitation code variables
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

volatile float t_profile_error;
volatile float Kp = 0.001;
//volatile float t_compressor_adapt[72] = {  //adapted 
//0.5,0.5,0.45,0.425,0.4,0.375,0.35,0.34,0.325,0.3,0.35,0.4,0.5,0.6,0.8,0.9,1.2,1.4,1.7,2,2.4,2.75,3,3.5,4,4.4,4.7,5,5.1,5.15,5.25,5.25,5.25,5.25,5.2,5.15,5.1,5,4.7,4.5,4.1,3.8,3.5,3.3,3.05,2.9,2.7,2.55,2.45,2.3,2.1,2,1.85,1.75,1.65,1.55,1.45,1.4,1.35,1.3,1.25,1.2,1.05,1,0.95,0.9,0.85,0.75,0.7,0.65,0.6,0.55
//};
	volatile float wc_dt = 100*two_pi*dt; //10 Hz delta angle for frequency emulation (carrier frequency)
	volatile float theta_c;
	volatile int theta_idx1;
	volatile int theta_idx2;
	volatile float theta_idx_act;
	volatile float m_interp;
 
	
//	float COST[7];
//	float FLUX[7];
	int index_min;
	int i;
	float COST_min;
//	float COST_km1;
//	float COST_improv;
	float lambda_s_star_mtpa_km1;
	
	volatile float lambda_s_star_sin_amp_2		=4.0f;
	volatile float lambda_s_star_sin_freq_2		=0.1f;
	volatile float lambda_s_star_sin_offset_2	=6.0f;
	
	volatile float tem_star_sin_amp_1 = 0.0f; ;
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


/* Dynamic Testing Macro  */
	volatile float 	dyn_macro_flag			= 1.0f;
	volatile float 	dyn_macro_cmd			= 0.0f;
	volatile int	dyn_macro_timer			= 0;
	volatile int	dyn_macro_pretrig		= 400;
	volatile float 	dyn_macro_lambda_star	= 0.35f;
	volatile float 	dyn_macro_omega_star    = 0.0f;
	volatile float  dyn_macro_torque_cmd    = 0.0f;
	volatile float  dyn_macro_torque_cmd_nxt= 0.0f;
	volatile float  dyn_macro_torque_slew	= 0.0001f;
	volatile float  dyn_macro_torque_offset = 0.0f;


/* HEX Overmodulation  */
	volatile float 	alpha;
	volatile float	alpha_e;
	volatile float	theta_v;
	volatile float 	mu;
	volatile float 	v_dqs_s_star_mag;
/* Test Variables */
	volatile int test_cnt=0;
	volatile float test_pwm=0.0f;
	volatile float test_3=0;
	volatile float test_4=0;
	volatile float test_5=0.0f;

/* Exit overmodulation*/
	struct	dq test1;
	struct	dq test2;
	struct	dq test3;	
	
/* Constant Voltage Excitation */
	volatile float v_exc_amp_2 = 00.0f;
	volatile float v_exc_freq_2 = 60.0f;
	volatile int   v_exc_count_2 = 0;
	volatile int   v_exc_count_1 = 0;

/* Chopper Circuit  */

	volatile int brake_status=0;
	volatile int brake_status_km1=1;
	
	
/* Improved Torque Line Model */

	volatile float Fit[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	volatile int coeff_index=0;
	volatile int curve_fit_index=0;
	volatile float X1 = tor_mod_coeff_30[0];
	volatile float X2 = tor_mod_coeff_30[1];
	volatile float X3 = tor_mod_coeff_30[2];
	volatile float X4 = tor_mod_coeff_30[3];
	volatile float X5 = tor_mod_coeff_30[4];
	volatile float X6 = tor_mod_coeff_30[5];
	volatile float Y1 = tor_mod_coeff_30[6];
	volatile float Y2 = tor_mod_coeff_30[7];
	volatile float Y3 = tor_mod_coeff_30[8];
	volatile float Y4 = tor_mod_coeff_30[9];
	volatile float Y5 = tor_mod_coeff_30[10];
	volatile float Y6 = tor_mod_coeff_30[11];
	volatile float Z1 = tor_mod_coeff_30[12];
	volatile float Z2 = tor_mod_coeff_30[13];
	volatile float Z3 = tor_mod_coeff_30[14];
	volatile float Z4 = tor_mod_coeff_30[15];
	volatile float Z5 = tor_mod_coeff_30[16];
	volatile float Z6 = tor_mod_coeff_30[17];
	volatile float W1 = tor_mod_coeff_30[18];
	volatile float W2 = tor_mod_coeff_30[19];
	volatile float W3 = tor_mod_coeff_30[20];
	volatile float W4 = tor_mod_coeff_30[21];
	volatile float W5 = tor_mod_coeff_30[22];
	volatile float W6 = tor_mod_coeff_30[23];
	
	volatile float a4 =0.0f;
	volatile float a5 =0.0f;
	volatile float a6 =0.0f;
	
	// new slope and offset in stationery reference frame
	//volatile float m_s =0.0f;	
	//volatile float b_s =0.0f;
	
	// new slope and offset in sync reference frame
	volatile float m_e =0.0f;	
	volatile float b_e =0.0f;
	
	volatile float dummy_tor1=0.0f;
	volatile float dummy_tor2=0.0f;
	volatile float dummy_tor3=0.0f;
	volatile float dummy_tor4=0.0f;
	//struct	dq point_1;
	//struct	dq point_2;
	struct  dq v_dqs_s_star_db_improve_2;
	struct  dq v_dqs_e_star_db_improve_2;
	
	volatile float tem_hat_kp1_cm_2=0.0f;
	volatile float z_d_imp=0.0f;
	volatile float z_q_imp=0.0f;
	volatile float root_1_imp=0.0f;
	volatile float root_2_imp=0.0f;
	
		// DB-DTFC Torque Mod selection var
	volatile float 	switch_to_new_model=0.0f;
	
	
	// updated db-dtfc vriables
	volatile float z_d_imp_ud=0.0f;
	volatile float z_q_imp_ud=0.0f;
	volatile float DB_A=0.0f;
	volatile float DB_B=0.0f;
	volatile float X1X2pY1Y2=0.0f;
	volatile float X2X2pY2Y2=0.0f;
	volatile float b_prime=0.0f;
	volatile float alpha_db=0.0f;
	volatile float beta_db=0.0f;
	volatile float gamma_db=0.0f;
	
	struct dq v_dqs_s_star_db_ud_2;

	
/* Enhanced Flux Obsr */
	volatile float X1_r;
	volatile float X1_i;
	volatile float X2_r;
	volatile float X2_i;
	volatile float X3_r;
	volatile float X3_i;
	volatile float Y1_r;
	volatile float Y1_i;
	volatile float Y2_r;
	volatile float Y2_i;
	volatile float Y3_r;
	volatile float Y3_i;
	
	volatile float X1_r_rotor;
	volatile float X1_i_rotor;
	volatile float X2_r_rotor;
	volatile float X2_i_rotor;
	volatile float X3_r_rotor;
	volatile float X3_i_rotor;
	volatile float Y1_r_rotor;
	volatile float Y1_i_rotor;
	volatile float Y2_r_rotor;
	volatile float Y2_i_rotor;
	volatile float Y3_r_rotor;
	volatile float Y3_i_rotor;
	volatile float Flux_Fit[6]={0,0,0,0,0,0};
	volatile float Flux_Fit_R[6]={0,0,0,0,0,0};// rotor reference frame
	volatile float Current_Fit[6]={0,0,0,0,0,0};
	volatile float Current_Fit_R[6]={0,0,0,0,0,0};// rotor reference frame
	struct  dq lambda_dqr_s_vm_full_kp1_2;
	struct  dq lambda_dqs_s_vm_full_kp1_2;
	struct  dq lambda_dqr_s_vm_full_2;
	struct  dq lambda_dqs_s_vm_full_2;
	volatile float tem_hat_vm_full_2=0;
	struct  dq lambda_dqr_s_err_full;
	struct  dq lambda_dqr_s_err_accum_full;
	struct  dq i_dqs_s_vm_full_kp1_2;
	struct  dq i_dqs_s_err_full;
    struct  dq i_dqs_s_err_accum_full;
    struct  dq i_dqs_s_vm_full_2;
    volatile float tem_hat_vm_kp1_2=0;
    
    struct  dq lambda_did_s;
        
   volatile int flag_pwm_clip=0;
        
   volatile int flag_tri_inj=0;    
        
              
 /* Space Vector Modulation */

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


struct dq Vtest;



/* Multilevel Space Vector Modulation */
#ifdef _MULTI_LEVEL_SVM


/* voltage reference*/
volatile float v_mag_multi_svm=0.0f;
volatile float v_phs_multi_svm=0.0f;
volatile float v_mag_multi_rev=0.0f;
volatile float v_phs_multi_rev=0.0f;
static const float sw_matrix_1[12]={1,0,1,0,0,1,1,0,1,0,0,1};
static const float sw_matrix_2[12]={1,0,0,1,0,1,1,0,0,1,0,1};
static const float sw_matrix_odd[12]={0,0,1,0,1,1,0,0,1,0,1,1};
static const float sw_matrix_even[12]={1,1,0,0,1,0,1,1,0,0,1,0};
static const float Vqs_rev_matrix[6]={0.3333,0.1667,-0.1667,-0.3333,-0.1667,0.1667};
static const float Vds_rev_matrix[6]={0,-0.2887,-0.2887,0,0.2887,0.2887};

volatile int sector_1;//determine the sector in the largest hexagon.
volatile int sector_2;//determine the sector in the separate hexagons.
struct  dq Vqds_rev;
volatile int flag_sector_2_even=2;
volatile int flag_sector_2_odd=2;

volatile float 	Tsv_a_u;
volatile float 	Tsv_b_u;
volatile float 	Tsv_0_u;
volatile float 	Tsv_a_v;
volatile float 	Tsv_b_v;
volatile float 	Tsv_0_v;
volatile float 	Tsv_a_w;
volatile float 	Tsv_b_w;
volatile float 	Tsv_0_w;
#endif


// process line trajectories
volatile float proc_trigger=0.0f;
volatile float proc_timer=0.0f;
volatile float proc_counter=0.0f;
volatile float phs_b_comp=0.0f;

// multilevel inverter
volatile int psc_pwm_ml=1;

volatile int lambda_count_2=0.0f;

volatile int vdc_test=0;
volatile float omega_test=0;

volatile float lambda_star_old=0.0f;

volatile int torque_control_2=0;//if this flag=0, torque control, otherwise speed control

volatile float braking_c1=-9.557;
volatile float braking_c2=17.42;
volatile float braking_c3=-16.93;

volatile float flux_c1=0.5715;
volatile float flux_c2=-1.5429;
volatile float flux_c3=1.3386;
volatile float flux_bt_max=0.71;


volatile int brake_cnt=0;
const int brake_cnt_max=741;



// voltage sensor
const float zero_cnt=1000000*dt;  // corresponding to 0 V
const float full_cnt=2100000*dt; // corresponding to 1250 V
const float vs_rate=1250.0/(full_cnt-zero_cnt);

volatile float volt_measured=0.0f;
volatile unsigned vs_count1=0;
volatile unsigned vs_cnt=0;
volatile unsigned vs_cnt_last=0;
volatile unsigned vs_delta_cnt=0;
volatile float vs_timer=0.0f;


// computation effort

unsigned timer1_start;
unsigned timer1_start_1;
unsigned timer1_end;

unsigned timer_total;


// new DTFC_algorithm
volatile int	solution_flag=0;// this is the flag to indicate if DTFC has possible solution
volatile int	solution_iterate_cnt=0;
volatile float	lambda_s_star_new=0.0;

volatile int fault_code=0;
	

volatile float lambda_s_star_cmd_new=0.0;



// variables of rotor flux trajectory
volatile float coeff_A1= 2*rr_hat_2*ls_hat_2/(ls_hat_2*lr_hat_2-lm_hat_2*lm_hat_2);
volatile float coeff_A2= (rr_hat_2*ls_hat_2/(ls_hat_2*lr_hat_2-lm_hat_2*lm_hat_2))*(rr_hat_2*ls_hat_2/(ls_hat_2*lr_hat_2-lm_hat_2*lm_hat_2));
volatile float coeff_B1=  coeff_A2*lm_hat_2*lm_hat_2/ls_hat_2/ls_hat_2;
volatile float coeff_A3 = 4.0/9.0/pole_pairs/pole_pairs*rr_hat_2*rr_hat_2;


volatile float coeff_A4 = 2.0/3.0/pole_pairs*rr_hat_2;
volatile float coeff_A5 = rr_hat_2/lr_hat_2/sigma_hat_2;
volatile float coeff_B2 = rr_hat_2*lm_hat_2/lr_hat_2/sigma_hat_2/ls_hat_2; 

volatile float stator_flux_traj=0.0;
volatile float stator_flux_X=0.0;


volatile int rotor_flux_traj_trigger=0;
volatile int rotor_flux_traj_cnt=0;
volatile int rotor_flux_traj_t1=100;
volatile int rotor_flux_traj_t2=500;
volatile int rotor_flux_traj_t3=900;
volatile int rotor_flux_traj_t4=15360*5;

volatile float rotor_flux_t1=0;
volatile float rotor_flux_t3=0;
volatile float rotor_flux_traj_slew=0.0001;
volatile float rotor_flux_filter_bw=coeff_A5/2/5;


volatile float rotor_flux_traj_km1=0.0;
volatile float rotor_flux_traj=0.0;
volatile float rotor_flux_traj_filtered_km1=0.0;
volatile float rotor_flux_traj_filtered=0.0;
volatile float rotor_flux_traj_filtered_km2=0.0;

volatile float rotor_flux_Y_km2=0.0;
volatile float rotor_flux_Y_km1=0.0;
volatile float rotor_flux_Y=0.0;

volatile float rotor_flux_traj_filtered_dot2=0;
volatile float rotor_flux_traj_filtered_dot1=0;
volatile float rotor_flux_traj_filtered_real=0;

volatile float stator_flux_X_test=0;
volatile float stator_flux_test=0;

volatile float tem_hat_kp1_filtered=0;
volatile float tem_hat_kp1_filtered_km1=0;
volatile float lambda_qdr_mag_square=0.0;

volatile float lambda_qdr_mag_square_f_km1;
volatile float lambda_qdr_mag_square_f;

volatile float stator_flux_old;

volatile float wsl_est;
volatile float root_margin;

volatile int rotor_flux_trigger_reset=0;
volatile float rotor_flux_delta=0;
volatile float rotor_flux_traj_gain=3.0;



// ROTOR FLUX CONTROL DB-DTFC
volatile int rotor_flux_dbdtfc_first_time=1; // yes=1, no =0 

struct 	dq i_dqs_e_rfdbdtfc;
struct 	dq lambda_dqs_e_rfdbdtfc;
volatile float lambda_dr_e_rfdbdtfc_f_km1=0.0;
volatile float lambda_dr_e_rfdbdtfc_f=0.0;
volatile float lambda_dr_e_rfdbdtfc=0.0;
volatile float lambda_dr_e_rfdbdtfc_cmd=0.0;
volatile float lambda_dr_e_rfdbdtfc_ref=0.0;
volatile float lambda_dqs_e_rfdbdtfc_cmd=0.0;

volatile int rotor_flux_dbdtfc_traj_gen_flag=1;


// alternate torque expression
volatile float tem_hat_kp1_2_alt1=0.0f;
volatile float tem_hat_kp1_2_alt2=0.0f;
volatile float tem_hat_kp1_2_alt3=0.0f;
volatile float tem_hat_kp1_2_alt4=0.0f;
volatile float tem_hat_kp1_2_alt5=0.0f;
volatile float tem_hat_kp1_2_alt6=0.0f;

volatile float tem_hat_kp1_2_alt1_filtered=0.0f;
volatile float tem_hat_kp1_2_alt2_filtered=0.0f;
volatile float tem_hat_kp1_2_alt0_filtered=0.0f;
volatile float Lm_o_Lr_est=0.0f;
volatile float sigma_t_Ls=0.0f;


volatile float 		   	lambda_dr_e_rfdbdtfc_cmd_reserve=0.0f;
volatile float		   	lamda_star_2_reserve=0.0f;
volatile float		   	tem_star_2_reserve=0.0f;




// MRAS system
volatile int Lm_MRAS_flag=0;
volatile int Lm_MRAS_flag_first=1; // first time get into the routine
volatile int Lm_MRAS_flag_reset=0;
struct dq lambda_dqr_r_cm_2_MRAS;
struct dq lambda_dqr_r_cm_2_kp1_MRAS;
struct dq lambda_qdr_r_cm_dot_2_MRAS;

struct dq lambda_dqr_r_hat_kp1_2_MRAS;
struct dq i_dqs_e_MRAS;
struct dq lambda_dqr_s_cm_2_kp1_MRAS;
struct dq lambda_dqr_e_cm_2_kp1_MRAS;

volatile float lambda_dqr_coherence_power_Lm=0.0;
volatile float lambda_dqr_coherence_power_Rr=0.0;
volatile float Rr_MRAS=0.3;
volatile float Lm_MRAS=0.03;

volatile float Lr_MRAS=Lm_MRAS+llr_hat_2;
volatile float Lm_adative_inc=0.0;
volatile float Lm_adative_inc_km1=0.0;
volatile float Rr_adative_inc=0.0;
volatile float Rr_adative_inc_km1=0.0;
volatile float tau_r_MRAS=Lr_MRAS/Rr_MRAS;
volatile float c_1_obsr_2_MRAS=c_1_obsr_2;
volatile float c_2_obsr_2_MRAS=c_2_obsr_2;
volatile float c_3_obsr_2_MRAS=Lm_MRAS*(1-exp(-dt/tau_r_MRAS));

volatile float K1_MRAS=0.00001;
volatile float K2_MRAS=0.001;
volatile float K3_MRAS=0.0000001;
volatile float K4_MRAS=0.00001;
volatile float lambda_dqr_r_ref_mag=0.0;
volatile float lambda_dqr_r_model_mag=0.0;
volatile float i_dqs_r_mag=0.0;


volatile float Lm_adative_inc_accu=0.0f;
volatile float Rr_adative_inc_accu=0.0f;
volatile float Lm_adative_inc_accu_km1=0.0f;
volatile float Rr_adative_inc_accu_km1=0.0f;



volatile int FOC_first_time=1;
volatile int lambda_r_star_select_2=1;

volatile float tem_hat_MRAS=0.0f;
volatile float tem_hat_MRAS_ref=0.0f;


// voltage sensor variables:
volatile int vs_cnt_A;
volatile int vs_cnt_B;
volatile int vs_cnt_C;
volatile int vs_cnt_last_A;
volatile int vs_cnt_last_B;
volatile int vs_cnt_last_C;
volatile int vs_delta_cnt_A;
volatile int vs_delta_cnt_B;
volatile int vs_delta_cnt_C;
volatile float volt_measured_VU;
volatile float volt_measured_WU;
