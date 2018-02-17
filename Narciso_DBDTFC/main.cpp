
/*************************************************************************/
/*						DB DTFC Control Program - main.cpp				 */
/*						v5.0											 */
/*						12/15/2011										 */
/*						Tim Obermann, Zachary Hurst, Brian Bradley		 */
/*************************************************************************/
//DP_v3_T_load used to do lock the rotor test, motion control on load side


/**************************************************************************/
/**************************************************************************/
/**                                                             		 **/
/**                                                             		 **/
/**     Define Definitions: These switches determing which sections		 **/
/**		of software get complied and loaded on the DSP.  Due to memory	 **/
/**		limitations of the SHARC DSP AIX uses not all sections can be    **/
/**		run simultaneously                                            	 **/
/**                                                             		 **/
/**                                                             		 **/
/**                                                             		 **/
/**************************************************************************/
/**************************************************************************/
//#define VDSP

// AIX Scope Functionality
//#define _AIXSCOPE

// Test Motor Functionality
#define _PWM_B

// Load Motor Functionality 
//#define _PWM_C

// Use Matlab Datalinker Functionality
#define _DATALINK

// Align d-axis with the initial theta
#define _ALIGNMENT

// Stator Current Observer
#define _STATOR_CURRENT_OBSERVER       

#define _I_Obs_1                                          

// Use ME 746 Style Motion Observer utilizing optical encoder
//#define _MOTION_OBSR_1 // Motion observer

// Shaft Torque Observer for AC Loading
//#define _TSHAFT_OBSR_1

// Use ME 746 Style State Feedback Controller or Axis 1
//#define _MOTION_SFB_1  // SFb control for load machine

// Use ME 746 Style State Feedback Controller or Axis 2
//#define _MOTION_SFB_2  // SFb control for test machine

// Use stator flux observer
//#define _STATOR_FLUX_OBSERVER

/**************************************** 
*****************************************/

// Use ME 746 Style INCREMENTAL Motion Observer utilizing optical encoder
//#define _INCR_MOTION_OBSR_1 // Motion observer

// Use ME 746 Style INCREMENTAL Motion Observer utilizing optical encoder
//#define _INCR_MOTION_OBSR_2

// Use ME 746 Style INCREMENTAL Motion Observer utilizing optical encoder for SHAFT TORQUE estimate
//#define _INCR_TSHAFT_OBSR_2  // Tshaft estimate

// Use ME 746 Style INCREMENTAL State Feedback Controller or Axis 2
//#define _INCR_MOTION_SFB_2  // SFb control for test machine

// Use ME 746 Style INCREMENTAL State Feedback Controller or Axis 1
//#define _INCR_MOTION_SFB_1  // SFb control for load machine

// Chirp Signal Disturbance Axis 2 (drive)
//#define _CHIRP

// Inject Noise disturbances
//#define _NOISE

// Square Wave Torque Command
//#define _SQUARE_TORQUE

// Sine Wave Current
//#define _SINE_WAVE

// Decoupling for Crank dependent Torque and/or DID
//#define _CRANK_TORQUE_SFB_DECOUP_2 

//Angle Dependent Loading
//#define _CRANK_LOADING 

//Use Drivecycle Trajectory
//#define _DRIVECYCLE 

// Use FOC torque Modulator on Axis 1
//#define _FOC_1

// Enable use of FOC for Axis 2 (test machine)
#define _FOC_2

// Enable use of DB DTFC for Axis 2 (test machine)
//#define _DBDTFC_2

// Enable use of DB Current Regulator for Axis 2 (test machine)
//#define _DBCR_2

// Space Vector Modulation
//#define _SVM

// Enable Overmodulation Scheme for DB DTFC 
//#define _OVERMOD_CIR_2
//#define _OVERMOD_HEX_2
//#define _OVERMOD_FLUX_DECREASE_2
//#define _OVERMOD_FLUX_DECREASE_TEST_2
//#define _OVERMOD_FLUX_PROP_2
//#define _OVERMOD_SIXSTEP_2
//#define _OVERMOD_PERPENDICULAR_2
//#define _OVERMOD_PERPENDICULAR_TEST_2
//#define _OVERMOD_MPC_2
//#define _TRAJ_OVM_2

//Enable Lagrange Multiplier Loss Minimized Operations for DB DTFC (this shares flux_cmd ==4.0f)
//#define _Lagrange_Opt

//Enable MTPA
//#define _MTPA

//Enable FOC MTPA (load machine)
//#define _FOC_2_MTPA

// Enable use of Voltage Sinusoidal Excitation of Axis 1
//#define _V_EXC_1

// Enable use of Voltage Sinusoidal Excitation of Axis 2
//#define _V_EXC_2

// Enable Triplen Harmonic Injection to maximize inverter DC bus utilization
//#define _TRIPLEN_INJ_2

// Enable Current Sensor Auto Zeroing in INIT State
//#define _SENSOR_AUTO_ZERO

// Enable Open Loop Flux Estimates
//#define _OPEN_LOOP_FLUX_OBSR

// Enable Macro to collect steady state op points
//#define _SS_DATA_COLLECT

// Enable dynamics collection macro
//#define _DYN_DATA_COLLECT

// Enable Data Logging
#define _DATA_LOGGING

// Enable D/A Output
//#define _DA_OUTPUT

// Enable PWM Comp Axis 1
//#define _INV_COMP_1

// Enable PWM Comp Axis 2
//#define _INV_COMP_2

// Enable Parameterized DB DTFC Model One of the two must be switched in
//#define _PARAMETERIZED_MODEL 		//online calc
#define _NON_PARAMETERIZED_MODEL	//initialized once in header


//BFB Pertubation Search Algorithm 
//#define _OPT_SEARCH

// Parameterized Hexagon
//#define _OVERMOD_HEX_2


/**************************************************************************/
/**************************************************************************/
/**     Include Main Header File for variable definitions				 **/
/**		                                                      			 **/
/**************************************************************************/
/**************************************************************************/
#include "main.h"
#include "WT_CurrentObserver.h"
/**************************************************************************/
/**************************************************************************/
/** name:	  control_ISR												 **/
/**																		 **/
/** function: Current loop @ 10 kHz										 **/
/**			  															 **/
/**			  This function executes every time step to perform control	 **/
/**			  functions of the the test stand.							 **/
/**			  															 **/
/**			  															 **/
/**************************************************************************/
/**************************************************************************/
CurrentObserver CO;
void control_ISR (irq_param){
 
    	        
    /**************************************************************************/
	/**************************************************************************/
	/**     Service AD interrupt flag and convert Values					 **/
	/**		                                                      			 **/
	/**************************************************************************/
	/**************************************************************************/
	AN0_15.ADready.ack();	
	ad_convert(AN8,AN15);
	AN16_31.ADready.ack();	
	ad_convert(AN16,AN31);
	
	
    /**************************************************************************/
	/**************************************************************************/
	/**     Calulate Line Currents  and DC Link Voltage   					 **/
	/**		                						 						 **/
	/**		 The current sensors offsets are set in the header file and      **/
	/**		 are tested for off line.                                        **/
	/**                                                                      **/
	/**		1 - Load Machine (Slot 6)   2 - Test Machine  (Slot 4)		 	 **/					                                
	/**		                						 						 **/
	/**************************************************************************/
	/**************************************************************************/
	

	// Line Current Calculation
	i_a_2 		= -(AN[AN08] + i_a_offset_2) * 1.768f;//1.9178f;//2.0f/15.0f; 		    	    // Scaling from Semikron Datasheet
	i_b_2 		= -(AN[AN12] + i_b_offset_2) * 1.768f;//1.9178f;//2.0f/15.0f ;			    	// Scaling from Semikron Datasheet
	i_c_2 		= -(i_a_2 + i_b_2);                         				// Scaling from Semikron Datasheet
	i_a_1 		= -(AN[AN16] + i_a_offset_1) * 1.768f;				// Scaling from LEM Datasheet (1.75)
	i_b_1 		= -(AN[AN20] + i_b_offset_1) * 1.6475f;				// Scaling from LEM Datasheet
	i_c_1 		= -(i_a_1 + i_b_1);				                            // Scaling from LEM Datasheet
	

	// Calculate DC Link Voltage
    v_dc_link = AN[AN14]*13.0f/2.0f*0.9831688f;						    //208.5f // Scaling from Semikron Datasheet
	
	// Phase Volstage Calculation 
	// Note: v_a_1, v_b_1, v_c_1, etc, are used elsewhere
	//v_ab_2_ad 	= -(AN[AN10] + v_ab_offset_2) * 10.0f-30.1087208f;//32.0f/5.0f;//6400.0f/223.0f; 			        // Scaling
	//v_ca_2_ad 	= -(AN[AN13] + v_ca_offset_2) * 10.0f-30.1436863f;//32.0f/5.0f;//6400.0f/223.0f;			   		// Scaling
	//v_bc_2_ad 	= -(AN[AN09] + v_bc_offset_2) * 10.0f-30.6443520f;//32.0f/5.0f;//6400.0f/223.0f;

	v_a_2_measured 	= -(AN[AN10] + v_ab_offset_2) * 10.0f-35.6087208f;//32.0f/5.0f;//6400.0f/223.0f; 			        // Scaling
	v_b_2_measured 	= -(AN[AN13] + v_ca_offset_2) * 10.0f-36.1436863f;//32.0f/5.0f;//6400.0f/223.0f;			   		// Scaling
	v_c_2_measured 	= -(AN[AN09] + v_bc_offset_2) * 10.0f-35.5443520f;//32.0f/5.0f;//6400.0f/223.0f;

	//v_a_sense_2 = (v_ab_2_ad-v_ca_2_ad)/3.0f;
	//v_b_sense_2 = -(2.0f*v_ab_2_ad+v_ca_2_ad)/3.0f;
	//v_c_sense_2 = (v_ab_2_ad+2*v_ca_2_ad)/3.0f;

	//v_n_2 = -1.0f*(v_a_sense_2 + v_b_sense_2 + v_c_sense_2);

	v_dqs_measured_2.q = two_thirds*(v_a_2_measured-0.5f*(v_b_2_measured+v_c_2_measured));
	v_dqs_measured_2.d = sqrt3_over_3*(v_c_2_measured-v_b_2_measured);
	//v_dqs_measured_2.d = two_thirds*(v_b_2_measured-0.5f*(v_a_2_measured+v_c_2_measured))*cosf(two_pi/6.0f);


	// Scaling	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     System Protection   					 						 **/
	/**		                						 						 **/
	/**		The system is placed in the stop state of the the following 	 **/
	/**		conditions are met.												 **/
	/**		                						 						 **/
	/**		                						 						 **/
	/**		Over Current - Any one of the line current exceeds max_current   **/
	/**		Over Voltage - DC Link Voltage exceeds max_voltage				 **/
	/**		Over Speed   - The axis speed exceeds omega_max					 **/
	/**		                						 						 **/
	/**************************************************************************/
	/**************************************************************************/

	// Over current Protection

	if(fabs(i_a_1) > max_current | fabs(i_b_1) > max_current | fabs(i_c_1) > max_current){ 
	    drive_flag = STOP;
	    test1 = 1;
	    }
	if(fabs(i_a_2) > max_current | fabs(i_b_2) > max_current | fabs(i_c_2) > max_current){ 
	    drive_flag = STOP;
	    test1 = 2;
	    }
	/*if(fabs(i_a_inv_2) > max_current | fabs(i_b_inv_2) > max_current | fabs(i_c_inv_2) > max_current){ 
	    drive_flag = STOP;
	    
	    }*/
	    
	// Over voltage Protection
	if(v_dc_link   > max_voltage){ 
	    drive_flag =STOP;
	    test1 = 3;
	    }
	    
	// Overspeed protection
	//if(fabs(omega_encoder) > omega_max){ 
	//    drive_flag = STOP;
	    
	 //   }
	// Calcualte dq currents in sationary frame
	
	
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Convert 3 Phase to DQ Plane					 					 **/
	/**	 						                     						 **/
	/**	 	Take the 3 Phase line currents and transforms them into the 	 **/
	/**	 	DQ plane using method outlined in ECE 711  						 **/
	/**	 						                     						 **/
	/**************************************************************************/
	/**************************************************************************/
	
	// Axis 1
	i_dqs_s_1.q = two_thirds*(i_a_1 - 0.5f*i_b_1 - 0.5f*i_c_1);
	i_dqs_s_1.d = sqrt3_over_3*i_c_1-sqrt3_over_3*i_b_1 ;
	// Axis 2
	//if(count_PWM == PWM_frequency_divisor)
	//{
		i_dqs_s_2.q = two_thirds*(i_a_2 - 0.5f*i_b_2 - 0.5f*i_c_2);
		i_dqs_s_2.d = sqrt3_over_3*i_c_2-sqrt3_over_3*i_b_2 ;
	//}
	//count_PWM++;

	

    /**************************************************************************/
	/**************************************************************************/
	/**     Stop State									 					 **/
	/**	 						                     						 **/
	/**	 	Disable PWM on AIX controller and do nothing.					 **/
	/**	 						                     						 **/
	/**************************************************************************/
	/**************************************************************************/
	if(drive_flag == STOP)
	{		
		iqe_2 = 0;
		iqe_1 = 0;
		omega_star_kp1_1 = 0;
		delta_theta_1 = 0;
		#ifdef _CHIRP
			chirp_q = 0;
			chirp_d = 0;
		#endif
		//PWM_B.HOLDENABLE = 0xFF;					// Force inverter off
		PWM_C.HOLDENABLE = 0xFF;   				    // Force inverter off
	//	PWM_D.HOLDENABLE = 0xFF;                    // PWM in load side from slot 6
	 	PWM_B.HOLDENABLE = 0xFF;                    // PWM in test side from slot 4
		// pwm_set_combined_dutycycle(PWM_B.PAIR0, 0.5f);
		// pwm_set_combined_dutycycle(PWM_B.PAIR1, 0.5f);
		// pwm_set_combined_dutycycle(PWM_B.PAIR2, 0.5f);
		// pwm_set_combined_dutycycle(PWM_C.PAIR0, 0.5f);
		 //pwm_set_combined_dutycycle(PWM_C.PAIR1, 0.5f);
		// pwm_set_combined_dutycycle(PWM_C.PAIR2, 0.5f);
		CO.clear();
		I_dq_s_A.clear();
	 }
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Init State									 					 **/
	/**	 	Zero out Current Sensors                   						 **/
	/**************************************************************************/
	/**************************************************************************/
	if(drive_flag == INIT)
	{
		
		//PWM_B.HOLDENABLE = 0xFF;  						    // Force inverter off
	    PWM_C.HOLDENABLE = 0xFF;   							// Force inverter off
	
	    //PWM_D.HOLDENABLE = 0xFF;                    // PWM in load side from slot 6
	 	PWM_B.HOLDENABLE = 0xFF;                    // PWM in test side from slot 4
    
    
		// Autozeroing Routing for LEM Sensors - Does not work TRO 12-07-2009

		#ifdef _SENSOR_AUTO_ZERO
			i_a_offset_2 += AN[AN08]  *2.0f/15.0f; 			// Scaling from Semikron Datasheet
			i_b_offset_2 += AN[AN12]  *2.0f/15.0f;			// Scaling from Semikron Datasheet
			//i_c_offset_1 += AN[AN18] *125.0f/8.0f/166.0f;			// Scaling from Semikron Datasheet
			i_a_offset_1 += AN[AN16] *2.0f/15.0f;				// Scaling from Semikron Datasheet
			i_b_offset_1 += AN[AN20] *2.0f/15.0f;				// Scaling from Semikron Datasheet
			//i_c_offset_2 += AN[AN15] *1.75f/20000.0f;				// Scaling from Semikron Datasheet
	
			/*i_a_offset_1 += AN[AN8]  *125.0f/8.0f/166.0f; 			// Scaling from Semikron Datasheet
			i_b_offset_1 += AN[AN9]  *125.0f/8.0f/166.0f;			// Scaling from Semikron Datasheet
			i_c_offset_1 += AN[AN10] *125.0f/8.0f/166.0f;			// Scaling from Semikron Datasheet
			i_a_offset_2 += AN[AN15] *1.75f/20000.0f;				// Scaling from Semikron Datasheet
			i_b_offset_2 += AN[AN23] *1.75f/20000.0f;				// Scaling from Semikron Datasheet
			i_c_offset_2 += AN[AN19] *1.75f/20000.0f;				// Scaling from Semikron Datasheet
			i_a_inv_offset_2 += AN[AN16]  *125.0f/8.0f/166.0f; 		// Scaling from Semikron Datasheet
			i_b_inv_offset_2 += AN[AN17]  *125.0f/8.0f/166.0f;		// Scaling from Semikron Datasheet
			i_c_inv_offset_2 += AN[AN18]  *125.0f/8.0f/166.0f;		// Scaling from Semikron Datasheet 	
			*/ 
			init_count -= 1;
			if(init_count == 0)										// Exit after counter is done
			 {
			    init_count = 166;
			    drive_flag = STOP;
			 }

		#endif
		
	    
		
		// Set torque command to zero incase SFb is not enabled
		//tem_star_2 = 0.0f; //This is done below under the READY flag

	 

		 drive_flag = STOP;   // Place drive in STOP state upon exiting
	}// End if drive_flag == INIT
	
  	
	/**************************************************************************/
	/**************************************************************************/
	/**     Encoder sampling							 					 **/
	/**	 	Calculates encoder theta and omega bar from encoder interface	 **/
	/**************************************************************************/
	/**************************************************************************/
	ENCOD_A.IRQ1.ack();
	ENCOD_A.TRG = true;														// Required for reading Encoder Position
	enc_cnt = ENCOD_A.POS_A - enc_cnt_0;									// Read encoder count
	delta_cnt = enc_cnt - last_cnt; 										// Calculated Difference from last count
	last_cnt = enc_cnt;														// Set Old count for next loop
	theta = (two_pi*(float)((ENCOD_A.POS_A - enc_cnt_0)%total_enc_cnt)/total_enc_cnt);//			// Calc Shaft Theta
	//ENCOD_A.ZEROPOS_A;	
	
	//ENCOD_A.IRQ1.ack();
	//ENCOD_A.TRG = true;
	enc_cnt_1 = -ENCOD_A.POS_A - enc_cnt_0_1;
	delta_cnt_1 = enc_cnt_1 - last_cnt_1; 										// Calculated Difference from last count
	last_cnt_1 = enc_cnt_1;
	theta_1 = (two_pi*(float)((-ENCOD_A.POS_A - enc_cnt_0_1)%total_enc_cnt)/total_enc_cnt);
	//ENCOD_A.TRG = false;
	// VITAL: do not assign ENCOD_A.POS_A to a variable.
	//theta_filtered = 0.9*theta_filtered + 0.1*theta;

	if(theta < 0.0f)	
	{													// Keep theta between 0 and 2*pi
		//theta_filtered += two_pi;
		theta += two_pi;
	}
	
	if(theta_1 < 0.0f)
	{
		theta_1 += two_pi;
	}

	
	// Decimated Omega Calculation to increase resolution	
	if(count_omega == delta_theta_mult)
	{
		delta_theta = theta - theta_km1;	//calc delta theta
		delta_theta_1 = theta_1 - theta_km1_1;
		
		if(delta_theta > pi){
		    delta_theta -= two_pi;}
		if(delta_theta < -pi){
		    delta_theta += two_pi;}
		    
		if(delta_theta_1 > pi){
		    delta_theta_1 -= two_pi;}
		if(delta_theta_1 < -pi){
		    delta_theta_1 += two_pi;}

		omega_encoder   = delta_theta*pwm_frequency/delta_theta_mult;
		omega_encoder_1 = delta_theta_1*pwm_frequency/delta_theta_mult;
		//omega_enc_filtered = 0.9*omega_enc_filtered + 0.1*omega_encoder;
		count_omega = 0;
		
		theta_km1 = theta; 					//store old theta
		theta_km1_1 = theta_1;
	}
	count_omega++;
	
	theta_r_2 = pole_pairs*theta;
	
	// Latch delay compensation
	theta_r_comp_2 = theta_r_2 + 1.8f*pole_pairs*delta_theta/delta_theta_mult;
	theta_r_comp_2 = fmod(theta_r_comp_2, two_pi);

// Calc electrical position of machine
	theta_r_1 = pole_pairs*(theta_1);// + 0.73f*delta_theta_1/delta_theta_mult);
	theta_r_1 = fmod(theta_r_1, two_pi);
	
	if(theta_r_1 < 0.0f)
	{
		theta_r_1 += two_pi;
	}
	
	// Latch Delay Compensation
	theta_r_comp_1 = theta_r_1;// + 0.12f*pole_pairs*delta_theta_1;
	
	// Synchronous Frame Current
	i_dqs_e_2 = rotate(i_dqs_s_2,-1.0f*theta_r_comp_2);
	i_dqs_e_1 = rotate(i_dqs_s_1,-1.0f*theta_r_comp_1);

	//v_dqs_sense_e_2 = rotate(v_dqs_sense_s_2, theta_r_comp_2);

	/*		
	// Decimated Motion Loop for Calc of Omega bar (1/10 PWM Frequency )
	if(motion_flag == 0){													// Evaluate this part at 1/10 speed
	 	motion_flag = 9;													// Set coundown var for next time
	 	delta_cnt_w_r += delta_cnt;											// Keep track of number of counts since last update
	 	w_bar = (float)delta_cnt_w_r*two_pi/total_enc_cnt* enc_frequency; 	// Calc omega bar
	 	rpm = (float)w_bar*rad_s2rpm;										// Convert to RPM for display
	 	delta_cnt_w_r = 0;		}											// Reset Accumulator 
	else{																	// If not the 10th time just update the delta count
	    motion_flag -= 1; 													// Decimate motion flag and wait for next time
	    delta_cnt_w_r += delta_cnt;											// Update motion count
	}
	*/
	


	#ifdef _I_Obs_1

			CO.update(ComplexVar(i_dqs_s_2.d ,i_dqs_s_2.q ),ComplexVar(v_dqs_s_star_2.d,v_dqs_s_star_2.q));
			E_norm_d_s_A = CO.get_E_norm_d_est();
			E_norm_q_s_A = CO.get_E_norm_q_est();

			// CO.update(ComplexVar(i_dqs_s_2.d ,i_dqs_s_2.q ),ComplexVar(v_dqs_measured_2.d,v_dqs_measured_2.q));
			// E_norm_d_s_M = CO.get_E_norm_d_est();
			// E_norm_q_s_M = CO.get_E_norm_q_est();
			
			// BO.update(T_com_A,E_norm_q_s_A,E_norm_d_s_A);
			// theta_r_hats_A = BO.get_theta_kp1();
			
			// w_r_hats_A = BO.get_omega_kp1();
			// CMO.update(theta_r_hats_A, T_com_A, w_r_hats_A);
		 //    w_rm_hats_A = CMO.get_omega_kp1()/C_P_A;
		 //    theta_r_hats_pulse_A = CMO.get_theta_pulse_kp1();

		//	I_dq_c_B = CO.get_I_dq_c_hat_kp1();	
	#endif
		




	
	/**************************************************************************/
	/**************************************************************************/
	/**     Motion Observer 1							 					 **/
	/**	 	Use postion based motion observer as shown ME 577				 **/
	/**	 	Reduced order Luenberger style observer							 **/
	/**************************************************************************/
	/**************************************************************************/
	#ifdef _MOTION_OBSR_1
		theta_err_1 = theta_1 - theta_hat_k_1; 		// Calc error of observer
		if(theta_err_1 > pi)
		{
			theta_err_1 -= two_pi;
		}
		if(theta_err_1 < -pi)
		{
			theta_err_1 += two_pi;
		}
	
		// Wrap Error due to effects of encoder wrapping
		if(theta_err_1 > pi)
			theta_err_1 -= two_pi;
		if(theta_err_1 < -pi)
			theta_err_1 += two_pi;
	
	
		// Luenberger Observer
		t_cff_obsr_1 = tem_star_1-tem_star_2;									// Torque Command Feedforward
	
		theta_err_accum_1 = theta_err_accum_1+t_obsr_1*theta_err_1;				// Accumulate error for Kiso term
		torq_obsr_1=(theta_err_accum_1*kiso_1+theta_err_1*kso_1); 				// Torq calculated from Kso and Kiso //Disturbance Torque Estimate
		
		// Estimated Acceleration (For Active Interia)
		omega_dot_kp1_1  = j_p_hat_inv_1*t_cff_obsr_1;
		omega_dot_kp1_1 += j_p_hat_inv_1*theta_err_accum_1*kiso_1;
		omega_dot_kp1_1 += j_p_hat_inv_1*theta_err_1*kso_1;
		// Provide filtered estimate to reduce noise for dyno application
		omega_dot_filt_kp1_1 = omega_dot_filt_kp1_1*0.99f+omega_dot_kp1_1*0.01f;
	
		// Sum up contributions to Omega Left
		omega_l_hat_kp1_1  = omega_l_hat_k_1;
		omega_l_hat_kp1_1 += t_obsr_1*j_p_hat_inv_1*t_cff_obsr_1;
		omega_l_hat_kp1_1 += t_obsr_1*j_p_hat_inv_1*theta_err_accum_1*kiso_1;
		omega_l_hat_kp1_1 += t_obsr_1*j_p_hat_inv_1*theta_err_1*kso_1;
	
		// Sum up Omega Right
		omega_r_hat_kp1_1 = theta_err_1*bo_1*j_p_hat_inv_1+omega_l_hat_kp1_1;
		omega_bar_hat_kp1_1 = 0.5f*(omega_r_hat_kp1_1+omega_r_hat_k_1);			// Calc omega bar hat (Averaging)
		theta_hat_kp1_1 = theta_hat_k_1 + t_obsr_1*omega_bar_hat_kp1_1;			// Calc next rotor position
		theta_hat_kp1_1= fmod(theta_hat_kp1_1,two_pi);							// Wrap theta hat values
		
		// Wrap Theta Values
		if(theta_hat_kp1_1 < 0.0f)
			theta_hat_kp1_1 += two_pi;
	
		// Roll back values for next calculation
		theta_hat_k_1 = theta_hat_kp1_1;
		//theta_err_1_km1 = theta_err_1;
		omega_l_hat_k_1 = omega_l_hat_kp1_1;
		omega_r_hat_k_1 = omega_r_hat_kp1_1;	
	#endif

	
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     SHAFT TORQUE Observer 1						 					 **/
	/**	 	Use postion based motion observer as shown ME 577				 **/
	/**	 	Reduced order Luenberger style observer							 **/
	/**************************************************************************/
	/**************************************************************************/

	
	/**************************************************************************/
	/**************************************************************************/
	/**     											 					 **/
	/**    Velocity command generation					 					 **/
	/**     											 					 **/
	/**     											 					 **/
	/**    This block generates velocity commands for axis 1 based off		 **/
	/**    the value of omega_star_select_2.			 					 **/
	/**     											 					 **/
	/**     											 					 **/
	/**    omega_star_select_2							 					 **/
	/**     											 					 **/
	/**     				0 - Provide a Zero Speed Reference				 **/
	/**     				1 - Provide omega_star_const_2 (Constant Speed)	 **/
	/**     				2 - Provide sinusoidal reference				 **/
	/**     				3 - Provide EPA75 Drive Cycle Reference			 **/
	/**     											 					 **/
	/**    Additionaly speed commands are slewed for smoother transistions	 **/
	/**    this helps to ensure the command is feasible. 					 **/
	/**     											 					 **/
	/**    cmd_gen_step - Counter for sinusoidal commands 					 **/
	/**     											 					 **/
	/**************************************************************************/
	/**************************************************************************/

	
	if(drive_flag == RUN)
	{ 
		
	
	    // Record old reference for Slewing
		//omega_star_2 = omega_star_kp1_2;
	    
		
		// Zero Speed Reference Mode
	    if (omega_star_select_2 == 0.0f)
	    {
	        omega_star_kp1_2 = 0.0f;
	    }
	    
	    // Constant Speed Mode
	    if (omega_star_select_2 == 1.0f)
	    {
	        omega_star_kp1_2 = omega_star_const_2;
	    }
	    
	    // Sinusoidal Reference
	    if (omega_star_select_2 == 2.0f)
	    {
	        omega_star_kp1_2 = omega_star_sin_amp_2*sinf(two_pi*omega_star_sin_freq_2*cmd_gen_step*dt)+omega_star_sin_offset_2;
	        // Increment Counter
			cmd_gen_step += 1;	
			if (cmd_gen_step >= pwm_frequency/omega_star_sin_freq_2)
			{
			    cmd_gen_step -= pwm_frequency/omega_star_sin_freq_2;
			}
			
	    }
	    /*
	    // Square Reference
	    if (omega_star_select_2 == 3.0f)
	    {
	    	omega_star_kp1_2 = omega_star_sq_amp_2*/
	    
	   /*******************************************************************************************************/
	    
	    //Drive Cycle Reference (Scaled EPA75)
	    
	   
   
	    
	} // end drive_flag == RUN
	
	if(drive_flag == STARTUP)
	{
		cmd_gen_step = 0;
		#ifdef _CHIRP
			flag_start_chirp = 0;
		#endif
	}	
	

	/**************************************************************************/
	/**************************************************************************/
	/**     SFb motion controller for machine 1 (Load Machine)				 **/
	/**	 	Input = omega_star_kp1_1										 **/
	/**	 	Output =  tem_star_1											 **/
	/**************************************************************************/
	/**************************************************************************/
	//tem_star_1 = 0.0f; 												// Set Toque command to zero incase SFb is not enabled

	if(drive_flag == RUN)
	{
		#ifdef _MOTION_SFB_1
			tem_star_1 = 0.0f; 												// Set Toque command to zero incase SFb is not enabled	
			omega_err_kp1_1 = -1.0f*(omega_star_kp1_1 - omega_l_hat_kp1_1); // -1.0 becuase machine is on opposite side of shaft
			omega_err_kp1_accum1_1 = omega_err_kp1_accum1_1+t_sfb_1*omega_err_kp1_1;
			omega_err_kp1_accum2_1 = omega_err_kp1_accum2_1+t_sfb_1*omega_err_kp1_accum1_1;
			
			// Saturate Out Integrators
			if (ksa_1*omega_err_kp1_accum1_1 > t_max_1)
				omega_err_kp1_accum1_1 = t_max_1 / ksa_1;
			if (ksa_1*omega_err_kp1_accum1_1 < -t_max_1)
				omega_err_kp1_accum1_1 = -t_max_1 / ksa_1;
			if (kisa_1*omega_err_kp1_accum2_1> t_max_1)
				omega_err_kp1_accum2_1 = t_max_1 /kisa_1;
			if (kisa_1*omega_err_kp1_accum2_1 < -t_max_1)
				omega_err_kp1_accum2_1 = -t_max_1 / kisa_1;
			
			// Calculate Torque Command
			tem_star_1 = t_cff_sfb_1+ba_1*omega_err_kp1_1;
			tem_star_1+= ksa_1*omega_err_kp1_accum1_1;
			tem_star_1+= kisa_1*omega_err_kp1_accum2_1;
				
			// Squared Law Feedback (Quadratic Fan type loads)
			/*if (omega_err_kp1_1 < 0.0f)
			tem_star_1 -= omega_err_sqared_1*omega_err_kp1_1*omega_err_kp1_1;
			if (omega_err_kp1_1 > 0.0f)
			tem_star_1 += omega_err_sqared_1*omega_err_kp1_1*omega_err_kp1_1;
			*/
		
			// Active Inertia
			//tem_star_1 -= omega_dot_filt_kp1_1*ja_1;
		

			#ifdef  _DRIVECYCLE 
				//Torque Side
				//Vehicle Inertia Term
				tem_star_1 += omega_dot_filt_kp1_1*drivecycle_ja_1;
				
				//Vehicle Aerodynamic Drag Term
				tem_star_1 += omega_err_kp1_1*omega_err_kp1_1*drivecycle_drag_1;
		
				//Velocity Sign Dependent Rolling Resistance Term
				if(omega_err_kp1_1 > 1.0f){tem_star_1 += drivecycle_resistance_1;}
				if(omega_err_kp1_1 < -1.0f){tem_star_1 -= drivecycle_resistance_1;}	
		
				//Torque Slew to Make Smooth Transitions for Drivecycle
				if((tem_star_1-tem_star_km1_1) > tem_star_slew_1){
					tem_star_1 = tem_star_km1_1 + tem_star_slew_1;   
				}
		
				if((tem_star_1-tem_star_km1_1) < -tem_star_slew_1){
					tem_star_1 = tem_star_km1_1 - tem_star_slew_1;   
				}
		
				//Roll over Values
				omega_err_km1_1 = omega_err_kp1_1;
				tem_star_km1_1 = tem_star_1;	
			#endif
		

		
		
			// Saturate tem_star at maximum value, This sets upper and lower torque limit
			if (tem_star_1 > t_max_1)
				tem_star_1 = t_max_1;
			if (tem_star_1 < -t_max_1)
				tem_star_1 = -t_max_1;	
		#endif

	
		/**************************************************************************/
		/**************************************************************************/
		/**     SFb motion controller for machine 2 (Test Machine)				 **/
		/**	 	Input = omega_star_kp1_2										 **/
		/**	 	Output =  tem_star_2											 **/
		/**************************************************************************/
		/**************************************************************************/
	
		#ifdef _MOTION_SFB_2
			tem_star_2 = 0.0f;												// Set torque command to zero incase SFb is not enabled		
			omega_err_kp1_2 = omega_star_kp1_2 - omega_encoder;//omega_encoder;//omega_r_hat_kp1_1;//
			//omega_star_kp1_2 = 0.0f*count*t_sfb_2;
			//count++;
		
			omega_err_kp1_accum1_2 = omega_err_kp1_accum1_2+t_sfb_2*omega_err_kp1_2;
			omega_err_kp1_accum2_2 = omega_err_kp1_accum2_2+t_sfb_2*omega_err_kp1_accum1_2;	
	
			// Saturate Integrators
			if (ksa_2*omega_err_kp1_accum1_2 > t_max_2)
				omega_err_kp1_accum1_2 = t_max_2 / ksa_2;
			if (ksa_2*omega_err_kp1_accum1_2 < -t_max_2)
				omega_err_kp1_accum1_2 = -t_max_2 / ksa_2;
			if (kisa_2*omega_err_kp1_accum2_2> t_max_2)
				omega_err_kp1_accum2_2 = t_max_2 /kisa_2;
			if (kisa_2*omega_err_kp1_accum2_2 < -t_max_2)
				omega_err_kp1_accum2_2 = -t_max_2 / kisa_2;
				
		
			// Calc T_Star
			//tem_star_2 =  t_cff_sfb_2;
			tem_star_2 = ba_2*omega_err_kp1_2;
			tem_star_2 += ksa_2*omega_err_kp1_accum1_2;
			tem_star_2 += kisa_2*omega_err_kp1_accum2_2;
		//	tem_star_2 += dyn_macro_torque_offset;
	
			//tem_star_sfb_total_2 = tem_star_2;  
	
			#ifdef _CRANK_TORQUE_SFB_DECOUP_2
				tem_star_2 += tem_star_1;
	
				//tem_star_2 = tem_crank; //for torque control mode
	
				//tem_star_2 += torq_obsr_2; //shaft torque estimate from shaft torque observer 
				//tem_star_2 += -torq_obsr_2a; //
		    #endif
	
		    // Saturate tem_star at set max torque	
			if (tem_star_2 > t_max_2)
				tem_star_2 = t_max_2;
			if (tem_star_2 < -t_max_2)
				tem_star_2 = -t_max_2;
		#endif	
		// end _MOTION_SFB_2


	
	
	//added by Brian 12/14/2011
	/**************************************************************************/
	/**************************************************************************/
	/**     Incremental Motion Observer 2									 **/
	/**	 	Use postion based motion observer as shown ME 577				 **/
	/**	 	Reduced order Luenberger style observer							 **/
	/**************************************************************************/
	/**************************************************************************/
		#ifdef _INCR_MOTION_OBSR_2
		//Calc incremental theta error
		delta_theta_err_2 = delta_theta - delta_theta_hat_kp1_2;

		// Luenberger Observer
		t_cff_obsr_2 = tem_star_2 - tem_star_1;											// Torque Command Feedforward
		delta_theta_err_accum_2 = delta_theta_err_accum_2 + delta_theta_err_2;			//Accumlate delta theta error
		delta_theta_err_accum2_2 = delta_theta_err_accum2_2 + delta_theta_err_accum_2*t_obsr_2;	//Accumulate the accumulated delta theta error
	
		//Saturate Integrators
	
		//Observer Controller Output (Disturbance Torque Estimate)
		torq_obsr_ctrl_2  = delta_theta_err_2*(bo_2/t_obsr_2); 
		torq_obsr_ctrl_2 += delta_theta_err_accum_2*kso_2;
		torq_obsr_ctrl_2 += delta_theta_err_accum2_2*kiso_2;
		// Estimated Acceleration 
		omega_dot_kp1_2  = j_drive_hat_inv_2*(t_cff_obsr_2 + torq_obsr_ctrl_2);
	
		// Omega Hat
		omega_hat_kp1_2  = omega_hat_k_2 + omega_dot_kp1_2*t_obsr_2;
	
		// Omega BAR Hat
		omega_bar_hat_kp1_2 = 0.5f*(omega_hat_kp1_2+omega_hat_k_2);			// Calc omega bar hat (Averaging)
	
		// Delta Theta Hat
		delta_theta_hat_kp1_2 = t_obsr_2*omega_bar_hat_kp1_2;		
	
		// Roll back values for next calculation
		delta_theta_hat_k_2 = delta_theta_hat_kp1_2;
		omega_hat_k_2 = omega_hat_kp1_2;
		omega_dot_hat_k_2 = omega_dot_hat_kp1_2;
		//omega_l_hat_kp1_1 = omega_hat_kp1_1; //used to conform to older code using omega(l/r)	
	#endif
	// end _INCR_MOTION_OBSR_2
	
	#ifdef _INCR_MOTION_OBSR_1
		//Calc incremental theta error
		delta_theta_err_1 = delta_theta_1 - delta_theta_hat_kp1_1;

		// Luenberger Observer
		t_cff_obsr_1 = -tem_star_2+tem_star_1;											// Torque Command Feedforward
		delta_theta_err_accum_1 = delta_theta_err_accum_1 + delta_theta_err_1;			//Accumlate delta theta error
		delta_theta_err_accum2_1 = delta_theta_err_accum2_1 + delta_theta_err_accum_1*t_obsr_1;	//Accumulate the accumulated delta theta error
	
		//Saturate Integrators
	
		//Observer Controller Output (Disturbance Torque Estimate)
		torq_obsr_ctrl_1  = delta_theta_err_1*(bo_1/t_obsr_1); 
		torq_obsr_ctrl_1 += delta_theta_err_accum_1*kso_1;
		torq_obsr_ctrl_1 += delta_theta_err_accum2_1*kiso_1;
		// Estimated Acceleration 
		omega_dot_kp1_1  = j_tot_hat_inv_1*(t_cff_obsr_1 + torq_obsr_ctrl_1);
	
		// Omega Hat
		omega_hat_kp1_1  = omega_hat_k_1 + omega_dot_kp1_1*t_obsr_1;
	
		// Omega BAR Hat
		omega_bar_hat_kp1_1 = 0.5f*(omega_hat_kp1_1+omega_hat_k_1);			// Calc omega bar hat (Averaging)
	
		// Delta Theta Hat
		delta_theta_hat_kp1_1 = t_obsr_1*omega_bar_hat_kp1_1;		
	
		// Roll back values for next calculation
		delta_theta_hat_k_1 = delta_theta_hat_kp1_1;
		omega_hat_k_1 = omega_hat_kp1_1;
		omega_dot_hat_k_1 = omega_dot_hat_kp1_1;
		//omega_l_hat_kp1_1 = omega_hat_kp1_1; //used to conform to older code using omega(l/r)	
	#endif
	// end _INCR_MOTION_OBSR_1
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Discrete Time Observer for Shaft Torque Estimate				 **/
	/**	 	Incremental position format										 **/
	/**	 	WAY too much encoder resolution noise (omega_bar_res = 7.8r/s)	 **/
	/**************************************************************************/
	/**************************************************************************/

	
	

	/**************************************************************************/
	/**************************************************************************/
	/**     Discrete Time SFb Motion Controller	[DRIVE Motor, 2]			 **/
	/**	 	Incremental position format										 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/
	#ifdef _INCR_MOTION_SFB_2
		delta_theta_star_2 = omega_star_kp1_2*t_sfb_2;
		//delta_theta_star_err_2 = delta_theta_star_2 - delta_theta;
		delta_theta_star_err_2 = delta_theta_star_2 - omega_hat_kp1_2*t_sfb_2;//delta_theta;// //use obsr feedback
		//delta_theta_star_err_2 = delta_theta_star_2 - omega_hat_kp1_1*t_sfb_2; // use filtered theta
		delta_theta_star_err_accum_2 = delta_theta_star_err_accum_2 + delta_theta_star_err_2;			//Accumlate delta theta error
		delta_theta_star_err_accum2_2 = delta_theta_star_err_accum2_2 + delta_theta_star_err_accum_2;	//Accumulate the accumulated delta theta error
	
		// Saturate Integrators
		if (ksa_2*delta_theta_star_err_accum_2 > t_max_2)
			delta_theta_star_err_accum_2 = t_max_2 / ksa_2;
		if (ksa_2*delta_theta_star_err_accum_2 < -t_max_2)
			delta_theta_star_err_accum_2 = -t_max_2 / ksa_2;
		if (kisa_2*t_sfb_2*delta_theta_star_err_accum2_2> t_max_2)
			delta_theta_star_err_accum2_2 = t_max_2 / (kisa_2*t_sfb_2);
		if (kisa_2*t_sfb_2*delta_theta_star_err_accum2_2 < -t_max_2)
			delta_theta_star_err_accum2_2= -t_max_2 / (kisa_2*t_sfb_2);
		
		// Calc T_Star
	
		//tem_star_2 =  t_cff_sfb_2;  //cff for motion controller
		tem_star_2 = (ba_2/t_sfb_2)*delta_theta_star_err_2;
		tem_star_2 += ksa_2*delta_theta_star_err_accum_2;
		tem_star_2 += kisa_2*t_sfb_2*delta_theta_star_err_accum2_2;
		//tem_star_2 += dyn_macro_torque_offset; //used as part of the dynamic data capture
		
		// Saturate tem_star at set max torque	
		if (tem_star_2 > t_max_2)
			tem_star_2 = t_max_2;
		if (tem_star_2 < -t_max_2)
			tem_star_2 = -t_max_2;
	#endif
	// end 	_INCR_MOTION_SFB_2
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Discrete Time SFb Motion Controller	[Load Motor, 1]				 **/
	/**	 	Incremental position format										 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/	

	#ifdef _INCR_MOTION_SFB_1
		delta_theta_star_1 = omega_star_kp1_1*t_sfb_1;
		delta_theta_star_err_1 = delta_theta_star_1 - delta_theta_1;//omega_hat_kp1_1*t_sfb_1;//omega_hat_kp1_1*t_sfb_1;//
		//delta_theta_star_err_1 = delta_theta_star_1 - delta_theta_hat_k_1; //use obsr feedback
		delta_theta_star_err_accum_1 = delta_theta_star_err_accum_1 + delta_theta_star_err_1;			//Accumlate delta theta error
		delta_theta_star_err_accum2_1 = delta_theta_star_err_accum2_1 + delta_theta_star_err_accum_1;	//Accumulate the accumulated delta theta error
	
		// Saturate Integrators
		if (ksa_1*delta_theta_star_err_accum_1 > t_max_1)
			delta_theta_star_err_accum_1 = t_max_1 / ksa_1;
		if (ksa_1*delta_theta_star_err_accum_1 < -t_max_1)
			delta_theta_star_err_accum_1 = -t_max_1 / ksa_1;
		if (kisa_1*t_sfb_1*delta_theta_star_err_accum2_1> t_max_1)
			delta_theta_star_err_accum2_1 = t_max_1 / (kisa_1*t_sfb_1);
		if (kisa_1*t_sfb_1*delta_theta_star_err_accum2_1 < -t_max_1)
			delta_theta_star_err_accum2_1= -t_max_1 / (kisa_1*t_sfb_1);
		
		// Calc T_Star
		
		tem_star_1 = (ba_1/t_sfb_1)*delta_theta_star_err_1;
		tem_star_1 += ksa_1*delta_theta_star_err_accum_1;
		tem_star_1 += kisa_1*t_sfb_1*delta_theta_star_err_accum2_1;
	
		// Saturate tem_star at set max torque	
		if (tem_star_1 > t_max_1)
			tem_star_1 = t_max_1;
		if (tem_star_1 < -t_max_1)
			tem_star_1 = -t_max_1;
		
	#endif
	// end _INCR_MOTION_SFB_1

	
	

	
	/**************************************************************************/
	/**************************************************************************/
	/**     Discrete Time SFb Motion Controller	[Load Motor, 1]				 **/
	/**	 	Incremental position format										 **/
	/**	 	      															**/
	/**		 f_0: starting frequency										**/
	/**    	 k_c: chirp counter												**/
    /**    	 Df_by_T_c: frequency difference (f_1 - f_0)/T_c, f1:1000 Hz, f0:0	 **/
	/**************************************************************************/
	/**************************************************************************/
	//if(count_PWM == PWM_frequency_divisor)
	//{


	#ifdef _NOISE
		if(drive_flag == RUN && flag_start_noise)
		{
			noise_q = noise_amp*(rand()*bit_32-0.5);
			//srand(5);
			noise_d = noise_amp*(rand()*bit_32-0.5);
		}
	#endif

	#ifdef _SQUARE_TORQUE
		
		k_square = pwm_frequency/10.0f; // Divisor is the square wave frequency
		if (square_torque_flag == true)
		{			
			if(count_square == k_square/2)
			{
				tem_star_2 = 0.1f;//lambda_r_star_2 = lambda_hat_2 + 0.0002f;//
			}
			if(count_square == k_square)
			{
				tem_star_2 = 0.0f;//lambda_r_star_2 = lambda_hat_2;//
				count_square = 0;
			}
			count_square++;
		}
		else
		{
			tem_star_2 = 0.0f;
		}
		
	#endif
	
		
	//} //end if count_PWM
		
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Online Parameter Adjustments									 **/
	/**	 																	 **/
	/**	 																	 **/
	/**************************************************************************/

		
	/**************************************************************************/
	/**************************************************************************/
	/**     Stator Current Observer											 **/
	/**	 																	 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/

	#ifdef _STATOR_CURRENT_OBSERVER	
		// Calc stationary frame voltages	
		//v_dqs_s_star_2.q= two_thirds*(v_a_2 - 0.5f*v_b_2 - 0.5f*v_c_2);
		//v_dqs_s_star_2.d= sqrt3_over_3*v_c_2-sqrt3_over_3*v_b_2 ;
		
		////////////
		// q-axis
		////////////
		// Roll back values 
		i_dqs_e_hat_2.q	= i_dqs_e_hat_kp1_2.q;
		// Roll error back	
		i_err_km1_2.q	= i_err_2.q;
		
		// Calc new error and accumulation of error	
		i_err_2.q		= i_dqs_e_2.q  - i_dqs_e_hat_2.q;//
		i_err_accum_2.q	= i_err_accum_2.q + dt*i_err_2.q;
	

		v_dqs_current_obsr_2.q  = k_3_obsr_2*i_err_2.q;
		v_dqs_current_obsr_2.q += k_4_obsr_2*i_err_accum_2.q;
		
		//v_dqs_current_obsr_2.q += chirp_q;//v_dqs_current_obsr_harm_decoup_2.q;

		// Calc voltage across eq. resistance
		if (torq_mod_select == 1)
		{
			v_dqs_current_obsr_2.q  += 	v_dqs_e_star_foc_2.q - dist_q;//v_dqs_sense_e_2.q;//-chirp_q;//
		}
		if (torq_mod_select == 2)
		{
			v_dqs_current_obsr_2.q += v_dqs_e_star_db_2.q - dist_q;//v_dqs_sense_e_2.q;//-chirp_q;//
		}
		if (torq_mod_select == 4)
		{
			v_dqs_current_obsr_2.q += v_dqs_e_star_dbcr_2.q - dist_q;//v_dqs_sense_e_2.q;//-chirp_q;//
		}


		//if(current_obs_low == false)
		//{
			v_dqs_current_obsr_2.q -= (ld_hat_2*i_dqs_e_2.d + lambda_hat_2)*pole_pairs*omega_encoder;//lq_hat_2*i_dqs_e_2.q*pole_pairs*omega_encoder;//omega_bar_hat_kp1_1;//
			// Apply voltage across eq. resistace with stator dynamics
			i_dqs_e_hat_kp1_2.q     = C_tau_2*(i_dqs_e_hat_2.q);//*cos(omega_encoder*pole_pairs*dt)-sin(omega_encoder*pole_pairs*dt)*i_dqs_e_hat_2.d);
			i_dqs_e_hat_kp1_2.q    += (1.0f/rs_hat_2)*(1.0f-C_tau_2)*(v_dqs_current_obsr_2.q);//*cos(omega_encoder*pole_pairs*dt)-v_dqs_current_obsr_2.d*sin(omega_encoder*pole_pairs*dt));
		//}
		/*else
		{
			speed_switch_2 = pole_pairs*omega_encoder*dt;
			phi = atan(omega_encoder*pole_pairs*tau_hat_2);
			C_freq_2 = 1.0f/sqrt(1.0f/(tau_hat_2*tau_hat_2)+omega_encoder*omega_encoder*pole_pairs*pole_pairs);
			i_dqs_e_hat_kp1_2.q = C_tau_2*cos(speed_switch_2)*i_dqs_e_hat_2.q;
			i_dqs_e_hat_kp1_2.q += (v_dqs_current_obsr_2.q+pole_pairs*omega_encoder*lambda_hat_2)/lq_hat_2*C_tau_2*C_freq_2*cos(speed_switch_2+phi);
			i_dqs_e_hat_kp1_2.q -= v_dqs_current_obsr_2.d/lq_hat_2*C_freq_2*C_tau_2*sin(speed_switch_2+phi);
			i_dqs_e_hat_kp1_2.q += i_dqs_e_hat_2.d*C_tau_2*sin(speed_switch_2);
		}*/
	
		////////////
		// d-axis
		////////////
		// Roll back values 
		i_dqs_e_hat_2.d	= i_dqs_e_hat_kp1_2.d;
		i_err_km1_2.d	= i_err_2.d;
	
		// Calc new error and accumulation of error		
		i_err_2.d		= i_dqs_e_2.d - i_dqs_e_hat_2.d;//
		i_err_accum_2.d	= i_err_accum_2.d + dt*i_err_2.d;
	
		v_dqs_current_obsr_2.d = k_3_obsr_2*i_err_2.d;
		v_dqs_current_obsr_2.d += k_4_obsr_2*i_err_accum_2.d;
		
		//v_dqs_current_obsr_2.d += chirp_d;//v_dqs_current_obsr_harm_decoup_2.d;

		// Calc voltage across eq. resistance
		if (torq_mod_select == 1)
		{
			v_dqs_current_obsr_2.d  += v_dqs_e_star_foc_2.d - dist_d;//v_dqs_sense_e_2.d;//-chirp_d;//
		}
		if (torq_mod_select == 2)
		{
			v_dqs_current_obsr_2.d += v_dqs_e_star_db_2.d - dist_d;//v_dqs_sense_e_2.d;//-chirp_d;//
		}
		if (torq_mod_select == 4)
		{
			v_dqs_current_obsr_2.d += v_dqs_e_star_dbcr_2.d - dist_d;//v_dqs_sense_e_2.d;//-chirp_d;//
		}
		
		//if(current_obs_low == false)
		//{
			v_dqs_current_obsr_2.d += lq_hat_2*i_dqs_e_2.q*pole_pairs*omega_encoder;//((ld_hat_2*i_dqs_e_2.d + lambda_hat_2)*pole_pairs*omega_encoder);//omega_bar_hat_kp1_1;//
					// Apply voltage across eq. resistance with stator dynamics
			i_dqs_e_hat_kp1_2.d      = C_tau_2*(i_dqs_e_hat_2.d);//*cos(omega_encoder*pole_pairs*dt)+i_dqs_e_hat_2.q*sin(omega_encoder*pole_pairs*dt));
			i_dqs_e_hat_kp1_2.d     += (1.0f/rs_hat_2)*(1.0f-C_tau_2)*(v_dqs_current_obsr_2.d);//*cos(omega_encoder*pole_pairs*dt)+v_dqs_current_obsr_2.q*sin(omega_encoder*pole_pairs*dt));
		//}
		/*else
		{
			i_dqs_e_hat_kp1_2.d = i_dqs_e_hat_2.d*C_tau_2*cos(speed_switch_2);
			i_dqs_e_hat_kp1_2.d += v_dqs_current_obsr_2.d/lq_hat_2*C_tau_2*C_freq_2*cos(speed_switch_2+phi);
			i_dqs_e_hat_kp1_2.d += (v_dqs_current_obsr_2.q-pole_pairs*omega_encoder*lambda_hat_2)/lq_hat_2*C_tau_2*C_freq_2*sin(speed_switch_2+phi);
			i_dqs_e_hat_kp1_2.d -= i_dqs_e_hat_2.q*C_tau_2*sin(speed_switch_2);
		}*/
		


	#endif
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Stator Flux Observer											 **/
	/**	 																	 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/
	
	#ifdef _STATOR_FLUX_OBSERVER
		
		// Roll Back Flux Observer
		lambda_dqs_r_hat_k_2.q = lambda_dqs_r_hat_kp1_2.q;
		lambda_dqs_r_hat_k_2.d = lambda_dqs_r_hat_kp1_2.d;
		
		// CURRENT MODEL QUANTITIES
		// q-axis current model
		lambda_dqr_r_cm_2.q  = i_dqs_e_2.q*lq_hat_2; //exp(-t_obsr_2/tau_r_hat_2)*lambda_dqr_r_cm_2.q;

		// d-axis current model
		lambda_dqr_r_cm_2.d  = i_dqs_e_2.d*ld_hat_2+lambda_hat_2;//exp(-t_obsr_2/tau_r_hat_2)*lambda_dqr_r_cm_2.d;

		// Stator Current Estimated Current in the Stationary Frame
		i_dqs_s_hat_kp1_2 = rotate(i_dqs_e_hat_kp1_2, theta_r_comp_2+1.8f*pole_pairs*delta_theta/delta_theta_mult);
		i_dqs_s_hat_2 	  = rotate(i_dqs_e_hat_2, theta_r_comp_2);
		
		// Rotate current back to stationary ref frame
		lambda_dqr_s_cm_2 = rotate(lambda_dqr_r_cm_2, theta_r_comp_2);
		
		// VOLTAGE MODEL QUANTITIES
		// Calc error for Gopinath style observer
		lambda_dqr_s_err.q = lambda_dqr_s_cm_2.q - lambda_dqs_s_vm_kp1_2.q;//-lambda_dqs_s_vm_kp1_2.q;//
		lambda_dqr_s_err.d = lambda_dqr_s_cm_2.d - lambda_dqs_s_vm_kp1_2.d;//-lambda_dqs_s_vm_kp1_2.d;//
		
		// Calc accum error for Gopinath style observer
		lambda_dqr_s_err_accum.q = lambda_dqr_s_err_accum.q + dt*lambda_dqr_s_err.q;
		lambda_dqr_s_err_accum.d = lambda_dqr_s_err_accum.d + dt*lambda_dqr_s_err.d;
		
		// Calc PI in Gopinath style observer
		lambda_dqr_s_int.q  = dt*k_1_obsr_2*lambda_dqr_s_err.q;
		lambda_dqr_s_int.q += dt*k_2_obsr_2*lambda_dqr_s_err_accum.q;
		
		lambda_dqr_s_int.d  = dt*k_1_obsr_2*lambda_dqr_s_err.d;
		lambda_dqr_s_int.d += dt*k_2_obsr_2*lambda_dqr_s_err_accum.d;
		
		// Calc accum error for Resistive Voltage Drop for Voltage model (feed forward)
		lambda_qds_s_vm_ff_2.q = dt*rs_hat_2*i_dqs_s_hat_kp1_2.q*0.5f;
		lambda_qds_s_vm_ff_2.q += dt*rs_hat_2*i_dqs_s_hat_2.q*0.5f;
		
		lambda_qds_s_vm_ff_2.d = dt*rs_hat_2*i_dqs_s_hat_kp1_2.d*0.5f;
		lambda_qds_s_vm_ff_2.d += dt*rs_hat_2*i_dqs_s_hat_2.d*0.5f;
	
		// Calc using voltage model q-axis
		lambda_dqs_s_vm_kp1_2.q  = lambda_dqs_s_vm_kp1_2.q;
		if (torq_mod_select == 1)
		{
			v_dqs_s_star_foc_2 = rotate(v_dqs_e_star_foc_2, theta_r_comp_2);
			lambda_dqs_s_vm_kp1_2.q += dt*(v_dqs_s_star_foc_2.q);//v_dqs_sense_s_2.q;//
		}
		if (torq_mod_select == 2)
		{
			v_dqs_s_star_db_2 = rotate(v_dqs_e_star_db_2, theta_r_comp_2);
			lambda_dqs_s_vm_kp1_2.q += dt*v_dqs_s_star_db_2.q;//v_dqs_sense_s_2.q;//
		}
		lambda_dqs_s_vm_kp1_2.q -= lambda_qds_s_vm_ff_2.q;                                     
		lambda_dqs_s_vm_kp1_2.q += lambda_dqr_s_int_km1.q;
		
		// Calc using voltage model d-axis
		lambda_dqs_s_vm_kp1_2.d  = lambda_dqs_s_vm_kp1_2.d;
		if (torq_mod_select == 1)
		{
			lambda_dqs_s_vm_kp1_2.d += dt*(v_dqs_s_star_foc_2.d);//v_dqs_sense_s_2.d;//
		}
		if (torq_mod_select == 2)
		{
			lambda_dqs_s_vm_kp1_2.d += dt*v_dqs_s_star_db_2.d;//v_dqs_sense_s_2.d;//
		}
		lambda_dqs_s_vm_kp1_2.d -= lambda_qds_s_vm_ff_2.d;                                     
		lambda_dqs_s_vm_kp1_2.d += lambda_dqr_s_int_km1.d;
		
		// Roll back PI quantities
		lambda_dqr_s_int_km1.q = lambda_dqr_s_int.q;
		lambda_dqr_s_int_km1.d = lambda_dqr_s_int.d;
		
	
	
		////////////////
		// Assign output
		////////////////
		lambda_dqs_s_hat_kp1_2.q = lambda_dqs_s_vm_kp1_2.q;
		lambda_dqs_s_hat_kp1_2.d = lambda_dqs_s_vm_kp1_2.d;
		
		lambda_dqs_r_hat_kp1_2 = rotate(lambda_dqs_s_hat_kp1_2,-1.0f*(theta_r_comp_2+1.8f*pole_pairs*delta_theta/delta_theta_mult));

	#endif
	// end _STATOR_FLUX_OBSERVER
	}// end if(run_flag == RUN)

	#ifdef _SINE_WAVE
		if (sine_wave_flag == true)
		{

			sine_q = sine_offset+sine_amp*cosf((float)two_pi*dt*sine_freq*count_sine);
			//sine_d = sine_offset+sine_amp*cosf((float)two_pi*dt*sine_freq*count_sine+posneg*0.5f*pi);
			count_sine++;
		}
		else
		{
			sine_q = 0.0f;
			sine_d = 0.0f;
			count_sine = 0;
		}
		#endif

	#ifdef _CHIRP 
		//flag_start_chirp = 1;	
	// Chirp Command Generation	
	if(drive_flag == RUN  && flag_start_chirp)
	{	
		k_c = k_c%((int)(T_c/dt));
	    chirp_q = chirp_amp*cosf(two_pi*k_c*dt*(f_0 + Df_by_T_c*k_c*dt)); //chirp amplitude
	    
	    // For a positive current rotation vector, use +0.5f*pi. Use -0.5f*pi for negative rotation.
	    // These rotations are all relative to machine 2.
	    chirp_d = chirp_amp*cosf(two_pi*k_c*dt*(f_0 + Df_by_T_c*k_c*dt)+posneg*0.5f*pi);
        k_c += 1.0f;  //increase chirp counter
        	dist_q = chirp_q;
	dist_d = chirp_d;
	}      	
	#endif
	// end _CHIRP

	
	//OPEN LOOP FLUX OBSERVER	
	
		
	/**************************************************************************/
	/**************************************************************************/
	/**     READY STATE														 **/
	/**	 																	 **/
	/**	 	Reset necessary variables and re-enable PWM for next run		 **/
	/**************************************************************************/
	/**************************************************************************/
	if(drive_flag == READY)
	{
	    
	    
	    // Reset all values
	    i_dqs_s_hat_2.q				= 0.0f;
	    i_dqs_s_hat_2.d				= 0.0f;
	    i_dqs_s_hat_kp1_2.q			= 0.0f;
	    i_dqs_s_hat_kp1_2.d			= 0.0f;
	    i_err_2.q					= 0.0f;
	    i_err_2.d					= 0.0f;
	    i_err_accum_2.q				= 0.0f;
	    i_err_accum_2.d				= 0.0f;
    	lambda_dqr_s_hat_kp1_2.q	= 0.0f;
    	lambda_dqr_s_hat_kp1_2.d	= 0.0f;
    	lambda_dqr_r_cm_2.q			= 0.0f;
    	lambda_dqr_r_cm_2.d			= 0.0f;
    	lambda_dqr_s_cm_2.q			= 0.0f;
    	lambda_dqr_s_cm_2.d			= 0.0f;
    	lambda_dqr_s_err.q			= 0.0f;
    	lambda_dqr_s_err.d			= 0.0f;
    	lambda_dqr_s_err_accum.q	= 0.0f;
    	lambda_dqr_s_err_accum.d	= 0.0f;
    	lambda_dqs_s_vm_2.q			= 0.0f;
    	lambda_dqs_s_vm_2.d			= 0.0f;
    	lambda_dqs_s_vm_kp1_2.q		= 0.0f;
    	lambda_dqs_s_vm_kp1_2.d		= 0.0f;
    	lambda_dqr_s_vm_kp1_2.q 	= 0.0f;
    	lambda_dqr_s_vm_kp1_2.d 	= 0.0f;
    	tem_hat_kp1_2				= 0.0f;
    	v_exc_count_2				= 0.0f;

    	
    	

    
	    tem_star_1 = 0.0f;   
		tem_star_2 = 0.0f;   
		// Allow PWM to operate again and set duty cycle of all pairs to 50%
		//PWM_B.HOLDENABLE = 0x00;
	 	PWM_C.HOLDENABLE = 0x00; 
//	 	PWM_D.HOLDENABLE = 0x00;
	 	PWM_B.HOLDENABLE = 0x00; 
	 	
	 	//PWM_B.HOLDENABLE = 0xFF;
	 	
		//pwm_set_combined_dutycycle(PWM_B.PAIR0, 0.5f);
		//pwm_set_combined_dutycycle(PWM_B.PAIR1, 0.5f);
		//pwm_set_combined_dutycycle(PWM_B.PAIR2, 0.5f);
	//	pwm_set_combined_dutycycle(PWM_C.PAIR0, 0.5f);
	//	pwm_set_combined_dutycycle(PWM_C.PAIR1, 0.5f);
	//	pwm_set_combined_dutycycle(PWM_C.PAIR2, 0.5f);
				
		// Reset Startup counter
		startup_count = 10*pwm_frequency;
		//run_count = 0;
		} // end if drive_flag == READY


	/**************************************************************************/
	/**************************************************************************/
	/**     STARTUP STATE													 **/
	/**	 																	 **/
	/**	 	Build up Flux in machines and prepare SFB controls				 **/
	/**************************************************************************/
	/**************************************************************************/
	if(drive_flag == STARTUP)
	{    	    
    	    

	    	
	    #ifdef _ALIGNMENT
			#ifdef _V_EXC_1
	    		v_dqs_s_star_vs_1.d = 0.0f;
	    		v_dqs_s_star_vs_1.q = 0;
	    		if ((startup_count < 5.0f*pwm_frequency) && (startup_count > 2.0f*pwm_frequency))
	    		{
	    			v_dqs_s_star_vs_1.d = 1.0f;
	    			v_dqs_s_star_vs_1.q = 0;
	    			
	    			ENCOD_A.IRQ1.ack();
					ENCOD_A.TRG = true;									// Required for reading Encoder Position
					enc_cnt_0_1 = -ENCOD_A.POS_A;						// Read encoder count
					
					DISPLAY.gotoxy(0,1);
	    			DISPLAY.putint_dec(enc_cnt_1);							
	    		}
	    	#endif
	    	#ifdef _V_EXC_2
	    		v_dqs_s_star_vs_2.d = 1.0f;
	    		v_dqs_s_star_vs_2.q = 0;
	    		
	    		ENCOD_A.IRQ1.ack();
				ENCOD_A.TRG = true;									// Required for reading Encoder Position
				enc_cnt_0 = ENCOD_A.POS_A;							// Read encoder count
				
	    		if (startup_count < 7.0f*pwm_frequency)
	    		{
	    			v_dqs_s_star_vs_2.d = 0.0f;
	    			v_dqs_s_star_vs_2.q = 0;
	    		}
	    	#endif
	    	#ifdef _FOC_1

				if ((startup_count < 5.0f*pwm_frequency) && (startup_count > 2.0f*pwm_frequency))
	    		{
	    			v_dqs_s_star_foc_1.d = 2.0f;
	    			v_dqs_s_star_foc_1.q = 0.0f;
	    			
	    			ENCOD_A.IRQ1.ack();
					ENCOD_A.TRG = true;									// Required for reading Encoder Position
					enc_cnt_0_1 = -ENCOD_A.POS_A;						// Read encoder count
	    			DISPLAY.gotoxy(0,1);
	    			DISPLAY.putint_dec(enc_cnt_1*10);
				}
				else
				{
					v_dqs_s_star_foc_1.d =0.0f;
					v_dqs_s_star_foc_1.q = 0.0f;
				}
	    		
			#endif
	    	#ifdef _FOC_2
	    		if (startup_count > 7.0f*pwm_frequency)
	    		{
					v_dqs_s_star_foc_2.d = 2.0f;
					v_dqs_s_star_foc_2.q = 0.0f;
				
					ENCOD_A.IRQ1.ack();
					ENCOD_A.TRG = true;									// Required for reading Encoder Position
					enc_cnt_0 = ENCOD_A.POS_A;							// Read encoder count
					DISPLAY.gotoxy(0,1);
	    			DISPLAY.putint_dec(enc_cnt*10);						
				}
				else //(startup_count < 2.6f*pwm_frequency)
	    		{
	    			v_dqs_s_star_foc_2.d = 0.0f;
	    			v_dqs_s_star_foc_2.q = 0.0f;
	    		}
			#endif
			
			#ifdef _DBDTFC_2
	    		if (startup_count > 7.0f*pwm_frequency)
	    		{
					v_dqs_s_star_db_2.d = 1.0f;
					v_dqs_s_star_db_2.q = 0.0f;
				
					ENCOD_A.IRQ1.ack();
					ENCOD_A.TRG = true;									// Required for reading Encoder Position
					enc_cnt_0 = ENCOD_A.POS_A;							// Read encoder count
					DISPLAY.gotoxy(0,1);
	    			DISPLAY.putint_dec(enc_cnt*10);						
				}
				else //(startup_count < 2.6f*pwm_frequency)
	    		{
	    			v_dqs_s_star_db_2.d = 0.0f;
	    			v_dqs_s_star_db_2.q = 0.0f;
	    			lambda_r_star_2 = lambda_hat_2;
	    		}
			#endif
			
			
		#endif
		

		tem_star_1 = 0.0f;   
		tem_star_2 = 0.0f;   

	    startup_count -=1;
	    // Exit when done
	    if (startup_count==0)
	    {
	    	#ifdef _FOC_1
		    // Reset Current Reg Integrators
		    	i_dqs_e_err_1.q 	= 0.0f;
		    	i_dqs_e_err_1.d     = 0.0f;
		   		i_dqs_e_err_int_1.q = 0.0f;
		    	i_dqs_e_err_int_1.d = 0.0f;
   	    	#endif
   	    
   	    
   	    	#ifdef _FOC_2
		    // Reset Current Reg Integrators
		    	i_dqs_e_err_2.q 	= 0.0f;
		    	i_dqs_e_err_2.d 	= 0.0f;
		    	i_dqs_e_err_int_2.q = 0.0f;
		    	i_dqs_e_err_int_2.d = 0.0f;
   	    	#endif

		    #ifdef _DBCR_2
		    	v_dqs_e_star_dbcr_2.q     = 0.0f;
		    	v_dqs_e_star_dbcr_2.d     = 0.0f;
		    	v_dqs_e_star_dbcr_int_2.q = 0.0f;
		    	v_dqs_e_star_dbcr_int_2.d = 0.0f;
		    #endif


   	    	// Hold Integrators in SFB controller
	    	#ifdef _MOTION_SFB_1
				omega_err_kp1_1 				= 0.0f;
				omega_err_kp1_accum1_1			= 0.0f;
				omega_err_kp1_accum2_1 			= 0.0f;
				tem_star_1 						= 0.0f;
				delta_theta_star_1 				= 0.0f;
				delta_theta_star_err_1 			= 0.0f;
				delta_theta_star_err_accum_1 	= 0.0f;
				delta_theta_star_err_accum2_1	= 0.0f;
			#endif
		
			// Hold Integrators in SFB controller
	    	#ifdef _MOTION_SFB_2
				omega_err_kp1_2 		= 0.0f;
				omega_err_kp1_accum1_2 	= 0.0f;
				omega_err_kp1_accum2_2 	= 0.0f;
				tem_star_2 				= 0.0f;
			#endif

			// Hold Integrators in Motion observer
			#ifdef _INCR_MOTION_OBSR_2
				delta_theta_star_1 				= 0.0f;
				delta_theta_err_2 			= 0.0f;
				delta_theta_err_accum_2 	= 0.0f;
				delta_theta_err_accum2_2 	= 0.0f;
				tem_star_1 					= 0.0f;
				tem_star_2 					= 0.0f;
			#endif
			// Hold Integrator in Motion observer
			#ifdef _INCR_MOTION_OBSR_1
				delta_theta_err_1 			= 0.0f;
				delta_theta_err_accum_1 	= 0.0f;
				delta_theta_err_accum2_1 	= 0.0f;
				tem_star_1 					= 0.0f;
				tem_star_2 					= 0.0f;
			#endif	
	    	drive_flag =RUN;
	    }

		
	} // end drive_flag == STARTUP
	

	
	/**************************************************************************/
	/**************************************************************************/
	/**     RUN STATE														 **/
	/**	 																	 **/
	/**	 	Fix command flux command to torque modulators					 **/
	/**************************************************************************/
	/**************************************************************************/	

	/**************************************************************************/
	/**************************************************************************/
	/**     START UP & RUN STATE											 **/
	/**	 																	 **/
	/**	 	Use torque modulators to excite machines						 **/
	/**************************************************************************/
	/**************************************************************************/	
	if(drive_flag == RUN)
	{
    
	
		#ifdef _FOC_2
	
			//for open loop position tracking estimation (Jp_hat tune...AC with no offset) 
	    	//tem_star_2 = -0.03f;
	    	// Calc electrical position of machine
			// Anti-windup
			v_dqs_e_star_foc_2.d = K_anti_windup_2*(v_dqs_e_star_foc_2.d - v_pre_sat_2.d);
			v_dqs_e_star_foc_2.q = K_anti_windup_2*(v_dqs_e_star_foc_2.q - v_pre_sat_2.q);
			

			//Limit d-axis Current
			if(i_dqs_e_star_2.d > i_d_max_2)
			{
				i_dqs_e_star_2.d = i_d_max_2;
			}
			if(i_dqs_e_star_2.d < -i_d_max_2)
			{
				i_dqs_e_star_2.d = -i_d_max_2;
			}
		
			// Calculate iq_e from the commanded torque

			iqe_2 = tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2*cosf((float)two_pi*dt*1.0f*count);
			
			//iqe_2 = tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2;
			i_dqs_e_star_2.q = iqe_2;//+0.25f*cos(two_pi*count*dt*100.0f);//tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2;//0.1f;//
			i_dqs_e_star_2.d = 0.0f;//0.0f;//-0.5f*tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2;//1.0f*cos(two_pi*count*dt*500.0f+0.5*pi);

			count++;
		
		    //Limit q-axis Current
			if(i_dqs_e_star_2.q > i_q_max_2)
			{
				i_dqs_e_star_2.q=i_q_max_2;
			}
			if(i_dqs_e_star_2.q < -i_q_max_2)
			{
				i_dqs_e_star_2.q=-i_q_max_2;
			}
						
			#ifdef _FOC_2_MTPA
				i_dqs_e_star_2.d = sqrt(tem_star_2*(two_thirds*(1.0f/pole_pairs)*lr_hat_2/(lm_hat_2*lm_hat_2)));
		
				//Limit Current
				if(i_dqs_e_star_2.d > i_d_max_2)
				i_dqs_e_star_2.d=i_d_max_2;
				// Calc rotor flux 
				lamda_dr_hat_2 = lamda_dr_hat_2*tau_r_den_2+tau_r_num_2*i_dqs_e_star_2.d*lm_hat_2;
				// Calc q axis current based off flux level
				i_dqs_e_star_2.q = i_dqs_e_star_2.d;
			    //Limit Current
				if(i_dqs_e_star_2.q > i_q_max_2)
				i_dqs_e_star_2.q=i_q_max_2;
				// Calculate Slip relation
				s_omega_e_star_2 = -1.0f*i_dqs_e_star_2.q*((one_o_tau_r_hat_2*lm_hat_2) / lamda_dr_hat_2);
		
			#endif

			//Synch frame current regulation starts here

			// Rotate to synch frame
			//i_dqs_s_hat_kp1_2 = rotate(i_dqs_e_hat_kp1_2, 1.0f*theta_r_comp_2);
			//i_dqs_e_star_2 = rotate(i_dqs_s_star_2, -1.0f*theta_r_comp_2);
		
			// Generate Error
			i_dqs_e_err_2.q= i_dqs_e_star_2.q - i_dqs_e_2.q;
			i_dqs_e_err_2.d= i_dqs_e_star_2.d - i_dqs_e_2.d;
			// Generate integral of error 
			i_dqs_e_err_int_2.q=i_dqs_e_err_int_2.q+i_dqs_e_err_2.q*dt;	//+w_bar*k_p_ireg_2*i_dqs_e_err_2.d;
			i_dqs_e_err_int_2.d=i_dqs_e_err_int_2.d+i_dqs_e_err_2.d*dt;	//-w_bar*k_p_ireg_2*i_dqs_e_err_2.q;
			// Calc command voltage
			v_dqs_e_star_foc_2.q = i_dqs_e_err_2.q*k_p_ireg_2+k_i_ireg_2*i_dqs_e_err_int_2.q; //-r_v_2*i_dqs_s_2.q;
			v_dqs_e_star_foc_2.d = i_dqs_e_err_2.d*k_p_ireg_2+k_i_ireg_2*i_dqs_e_err_int_2.d; //-r_v_2*i_dqs_s_2.q;

		
			// Add Pole Cancellation and BEMF Decoupling for CVCR
			v_dqs_e_star_foc_2.q += i_dqs_e_err_int_2.d*k_p_ireg_2*pole_pairs*omega_encoder;//omega_bar_hat_kp1_2;//omega_enc_filtered;//omega_l_hat_kp1_1;
			v_dqs_e_star_foc_2.q += lambda_hat_2*pole_pairs*omega_encoder;//omega_bar_hat_kp1_2;//omega_enc_filtered;//omega_l_hat_kp1_1;//
			//v_dqs_e_star_foc_2.q += chirp_q;

			v_dqs_e_star_foc_2.d -= i_dqs_e_err_int_2.q*k_p_ireg_2*pole_pairs*omega_encoder;//omega_bar_hat_kp1_2;//omega_enc_filtered;//omega_l_hat_kp1_1;
			//v_dqs_e_star_foc_2.d += chirp_d;
			v_pre_sat_2.d = v_dqs_e_star_foc_2.d;
			v_pre_sat_2.q = v_dqs_e_star_foc_2.q;

			// Saturate voltage command, prioritizing d-axis voltage
			//v_dc_link = v_dc_link*0.35f;//0.4;//
			
			if( v_dqs_e_star_foc_2.d > v_dc_link)
			{
				v_dqs_e_star_foc_2.d = v_dc_link;
			}
			if( v_dqs_e_star_foc_2.d < -v_dc_link)
			{
				v_dqs_e_star_foc_2.d = -v_dc_link;
			}


			v_q_remnant_2 = sqrt(v_dc_link*v_dc_link - v_dqs_e_star_foc_2.d*v_dqs_e_star_foc_2.d);

			if( v_dqs_e_star_foc_2.q > v_q_remnant_2)
			{
				v_dqs_e_star_foc_2.q = v_q_remnant_2;
			}
			if( v_dqs_e_star_foc_2.q < -v_q_remnant_2)
			{
				v_dqs_e_star_foc_2.q = -v_q_remnant_2;
			}
			

			// Rotate voltage back to sationary frame
			v_dqs_s_star_foc_2 = rotate(v_dqs_e_star_foc_2, theta_r_comp_2);

			/*v_phs_svm=phase_in_two_pi(v_dqs_s_star_foc_2.q,v_dqs_s_star_foc_2.d);

			sector = (int)ceil(v_phs_svm/pi_over_3);

			switch(sector)
			{
				case 1:
				
					v_ref.q = sqrt3_over_2/sinf(v_phs_svm+pi_over_3);
					break;
				
				case 2:
				
					v_ref.q = sqrt3_over_2/sinf(v_phs_svm);
					break;
				
				case 3:
				
					v_ref.q = sqrt3_over_2/sinf(v_phs_svm-pi_over_3);
					break;
				
				case 4:
				
					v_ref.q = -sqrt3_over_2/sinf(v_phs_svm+pi_over_3);
					break;
				
				case 5:
				
					v_ref.q = -sqrt3_over_2/sinf(v_phs_svm);
					break;
				
				case 6:
				
					v_ref.q = -sqrt3_over_2/sinf(v_phs_svm-pi_over_3);
					break;
				
			}

			v_ref.q = v_ref.q*v_dc_link;
			v_ref.d = 0.0f;

			v_ref = rotate(v_ref, -theta_r_comp_2);

			v_ref_mag = sqrt(v_ref.q*v_ref.q+v_ref.d*v_ref.d);

			if( v_dqs_e_star_foc_2.d > v_ref_mag)
			{
				v_dqs_e_star_foc_2.d = v_ref_mag;
			}
			if( v_dqs_e_star_foc_2.d < -v_ref_mag)
			{
				v_dqs_e_star_foc_2.d = -v_ref_mag;
			}


			v_q_remnant_2 = sqrt(v_ref_mag*v_ref_mag - v_dqs_e_star_foc_2.d*v_dqs_e_star_foc_2.d);

			if( v_dqs_e_star_foc_2.q > v_q_remnant_2)
			{
				v_dqs_e_star_foc_2.q = v_q_remnant_2;
			}
			if( v_dqs_e_star_foc_2.q < -v_q_remnant_2)
			{
				v_dqs_e_star_foc_2.q = -v_q_remnant_2;
			}

			v_dqs_s_star_foc_2 = rotate(v_dqs_e_star_foc_2, theta_r_comp_2);*/

			//v_dc_link = v_dc_link/0.35f;//0.4f;//


		#endif

	

		#ifdef _FOC_1

			// Anti-windup
			v_dqs_e_star_1.d = K_anti_windup_2*(v_dqs_e_star_1.d - v_pre_sat_1.d);
			v_dqs_e_star_1.q = K_anti_windup_2*(v_dqs_e_star_1.q - v_pre_sat_1.q);
			
			//theta_r_comp_1 = fmod(theta_r_comp_1, two_pi);
			
			// Calc D axis current
			i_dqs_e_star_1.d = 0.0f;//lamda_star_1*(1.0/lm_hat_1);
			//Limit Current
			if(i_dqs_e_star_1.d > i_d_max_1)
			{
				i_dqs_e_star_1.d=i_d_max_1;
			}
			// Calc rotor flux 
			//lamda_dr_hat_1 = lamda_dr_hat_1*tau_r_den_1+tau_r_num_1*i_dqs_e_star_1.d*lm_hat_1;
			
			// Calc q axis current based off flux level
			iqe_1 = tem_star_1*two_thirds*(1.0f/pole_pairs)/lambda_hat_1;
			
			i_dqs_e_star_1.q = iqe_1;
		    //Limit Current
			if(i_dqs_e_star_1.q > i_q_max_1)
			{
				i_dqs_e_star_1.q=i_q_max_1;
			}
			if(i_dqs_e_star_1.q < -i_q_max_1)
			{
				i_dqs_e_star_1.q=-i_q_max_1;
			}
		/*	// Calculate Slip relation
			s_omega_e_star_1 = -1.0f*i_dqs_e_star_1.q*((one_o_tau_r_hat_1*lm_hat_1) / lamda_dr_hat_1); //((trimmer4+1)*1.5f);
			// Calculate new reference frame
			theta_s_star_1 = theta_s_star_1 + s_omega_e_star_1*dt;
			//wrap theta s
			theta_s_star_1 = fmod(theta_s_star_1,two_pi);
			// Calc desired reference frame
			theta_rf_star_1 =theta_s_star_1 + theta_r_1;
			//wrap theta reference star
			theta_s_star_1 = fmod(theta_s_star_1,two_pi);*/
		
			//Synch frame current regulation starts here

			/*// Rotate to synch frame
			i_dqs_e_1 = rotate(i_dqs_s_1,-1.0f*theta_r_comp_1);*/

			// Generate Error
			i_dqs_e_err_1.q=i_dqs_e_star_1.q-i_dqs_e_1.q;
			i_dqs_e_err_1.d=i_dqs_e_star_1.d-i_dqs_e_1.d;
			// Generate integral of error 
			i_dqs_e_err_int_1.q=i_dqs_e_err_int_1.q+i_dqs_e_err_1.q*dt;	//+w_bar*k_p_ireg_1*i_dqs_e_err_1.d;
			i_dqs_e_err_int_1.d=i_dqs_e_err_int_1.d+i_dqs_e_err_1.d*dt;	//-w_bar*k_p_ireg_1*i_dqs_e_err_1.q;
			// Calc command voltage
			v_dqs_e_star_1.q += i_dqs_e_err_1.q*k_p_ireg_1+k_i_ireg_1*i_dqs_e_err_int_1.q; //-r_v_1*i_dqs_s_1.q;
			v_dqs_e_star_1.d += i_dqs_e_err_1.d*k_p_ireg_1+k_i_ireg_1*i_dqs_e_err_int_1.d; //-r_v_1*i_dqs_s_1.q;
		/*	// Add Cross Coupling Voltages
			v_dqs_e_star_1.q += i_dqs_e_1.d*(-1.0f*omega_r_hat_kp1_1)*ls_prime_hat_1;
			v_dqs_e_star_1.d -= i_dqs_e_1.q*(-1.0f*omega_r_hat_kp1_1)*ls_prime_hat_1;
			*/
			
			// Add Pole Cancellation and Cross-Coupling Decoupling for CVCR
			v_dqs_e_star_1.q += i_dqs_e_err_int_1.d*k_p_ireg_1*pole_pairs*omega_encoder_1;//omega_bar_hat_kp1_1;//omega_enc_filtered;//omega_l_hat_kp1_1;
			v_dqs_e_star_1.q += lambda_hat_1*pole_pairs*omega_encoder_1;//omega_enc_filtered;//omega_encoder;//omega_l_hat_kp1_1;//
			v_dqs_e_star_1.d -= i_dqs_e_err_int_1.q*k_p_ireg_1*pole_pairs*omega_encoder_1;//omega_enc_filtered;//omega_encoder;//omega_l_hat_kp1_1;
			
		/*	// Add VTP
			v_dqs_e_star_1.q -= r_v_1*i_dqs_e_1.q;
			v_dqs_e_star_1.d -= r_v_1*i_dqs_e_1.d;
			*/

			v_pre_sat_1.d = v_dqs_e_star_1.d;
			v_pre_sat_1.q = v_dqs_e_star_1.q;

			// Saturate voltage command, prioritizing d-axis voltage
			if( v_dqs_e_star_1.d > v_dc_link)
			{
				v_dqs_e_star_1.d = v_dc_link;
			}
			if( v_dqs_e_star_1.d < -v_dc_link)
			{
				v_dqs_e_star_1.d = -v_dc_link;
			}


			v_q_remnant_1 = sqrt(v_dc_link*v_dc_link - v_dqs_e_star_1.d*v_dqs_e_star_1.d);

			if( v_dqs_e_star_1.q > v_q_remnant_1)
			{
				v_dqs_e_star_1.q = v_q_remnant_1;
			}
			if( v_dqs_e_star_1.q < -v_q_remnant_1)
			{
				v_dqs_e_star_1.q = -v_q_remnant_1;
			}

			// Rotate voltage back to sationary frame
			v_dqs_s_star_foc_1 = rotate(v_dqs_e_star_1,theta_r_comp_1);
	


		#endif

	
		#ifdef _V_EXC_1 // V/F control of Load Machine
	
			v_exc_count_1 += 1;
		
			//v_exc_count_1 += 2*pi*f*dt;
				
		  	v_dqs_s_star_vs_1.q = v_exc_amp_1*cos((float)v_exc_freq_1*v_exc_count_1*dt*2*pi);
		  	v_dqs_s_star_vs_1.d = v_exc_amp_1*cos((float)v_exc_freq_1*v_exc_count_1*dt*2*pi+0.5*pi);
  		
  				// Calc electrical position of machine
			theta_r_1 = pole_pairs*theta_1;
	
			// Latch delay compensation
			theta_r_comp_1 = theta_r_1;// + 0.8f*delta_theta_1;
			// Wrap theta_r_comp_1
			theta_r_comp_1 = fmod(theta_r_comp_1, two_pi);
	
	  		//v_dqs_s_star_vs_1 = rotate(v_dqs_e_star_vs_1, 1.0f*theta_r_comp_1); 

	     #endif	
	
 
		
		//v_dqs_s_star_1.q = 0.0f;
		//v_dqs_s_star_1.d = 0.0f;


	


		
	
		#ifdef _V_EXC_2 // V/F control of Test Machine
	
			v_exc_count_2 += 1;
			
	  	  	v_dqs_e_star_vs_2.q = v_exc_amp_2;//*cos((float)v_exc_freq_2*v_exc_count_2*dt*2*pi);
	  		v_dqs_e_star_vs_2.d = 0.0f;//v_exc_amp_2*cos((float)v_exc_freq_2*v_exc_count_2*dt*2*pi+0.5*pi);
		
	  		// Calc electrical position of machine
			/*theta_r_2 = two_pi*((float)(((ENCOD_A.POS_A-enc_cnt_0)*pole_pairs)%total_enc_cnt)/total_enc_cnt);
			if(theta_r_2 > two_pi)
			{
				theta_r_2 -= two_pi;
			}
			if(theta_r_2 < 0.0f)
			{
				theta_r_2 += two_pi;
			}*/
			

	
			// Latch delay compensation
			//theta_r_comp_2 = theta_r_2;// + 0.8f*delta_theta;
			// Wrap theta_r_comp_2
			//theta_r_comp_2 = fmod(theta_r_comp_2, two_pi);
	
	  		v_dqs_s_star_vs_2 = rotate(v_dqs_e_star_vs_2, theta_r_comp_2); 		

		#endif	

		
		
			

	
	

		
	#ifdef _DBDTFC_2
	
		// Torque calculation
		tem_hat_kp1_2 = c_3_obsr_2*(lambda_dqs_r_hat_kp1_2.d*i_dqs_e_hat_kp1_2.q - lambda_dqs_r_hat_kp1_2.q*i_dqs_e_hat_kp1_2.d);//lambda_hat_2*i_dqs_e_hat_kp1_2.q;//
		
	/*	lambda_r_2  = lambda_dqs_r_hat_kp1_2.d*lambda_dqs_r_hat_kp1_2.d;
		lambda_r_2 += lambda_dqs_r_hat_kp1_2.q*lambda_dqs_r_hat_kp1_2.q;
		lambda_r_2 = sqrt(lambda_r_2);	*/
		
		lambda_r_star_2 = lambda_hat_2;// - tem_star_2*0.001;//lambda_r_2;//
		
		/**************************************************************************/
		/**************************************************************************/
		/**																		 **/
		/**     			DETERMINE VOLT-SEC Solution							 **/
		/**	 																	 **/	
		/**************************************************************************/
		/**************************************************************************/

		// Rotate Rotor Flux Vector into stator flux aligned synch reference frame 
		//lambda_dqr_e_2 = lambda_dqs_r_hat_kp1_2;//rotate(lambda_dqs_s_hat_kp1_2, -1.0f*theta_r_comp_2);


		// Calculate Delta Torque Term
		//tem_star_2 = 0.0f;//chirp;
		delta_tem_2 = tem_star_2-tem_hat_kp1_2;
		
		
		// Calculate slope of torque line (SPMSM, m = 0)
		m_2 = 0.0f;//-c_1_obsr_2*lambda_dqs_r_hat_kp1_2.q; //lambda_dqr_e_2.q / lambda_dqr_e_2.d;
		

		// Calculate Intercept of Torque Line (SPMSM)
		b_2  = 1.0f/c_3_obsr_2*delta_tem_2*ld_hat_2/lambda_hat_2*pwm_frequency;
		b_2 += pole_pairs*omega_encoder*lambda_dqs_r_hat_kp1_2.d; //omega_bar_hat_kp1_1*lambda_dqr_e_2.d;//
		b_2 += lambda_dqs_r_hat_kp1_2.q/tau_hat_2;
	

		
		//lambda_r_k_q  = lambda_dqs_r_hat_k_2.q;//*(1.0f-dt/tau_hat_2);
		//lambda_r_k_q += b_2*dt;

//
		v_dqs_e_star_db_2.q  = b_2+dist_q;

		v_dqs_e_star_db_2.d  = (lambda_r_star_2-lambda_dqs_r_hat_k_2.d)*pwm_frequency;// + 1.0f/tau_hat_2);
		v_dqs_e_star_db_2.d -= pole_pairs*omega_encoder*lambda_dqs_r_hat_k_2.q;
		v_dqs_e_star_db_2.d += (lambda_dqs_r_hat_k_2.d-lambda_hat_2)/tau_hat_2+dist_d;

//
		v_dqs_s_star_db_2 = rotate(v_dqs_e_star_db_2, theta_r_comp_2 + 1.8f*pole_pairs*delta_theta/delta_theta_mult);
	/*
			// Saturate voltage command, prioritizing d-axis voltage
			if( v_dqs_s_star_db_2.d > v_dc_link*0.2f)
			{
				v_dqs_s_star_db_2.d = v_dc_link*0.2f;
			}
			if( v_dqs_s_star_db_2.d < -v_dc_link*0.2f)
			{
				v_dqs_s_star_db_2.d = -v_dc_link*0.2f;
			}


			v_q_remnant_2 = sqrt(v_dc_link*0.2f*v_dc_link*0.2f - v_dqs_s_star_db_2.d*v_dqs_s_star_db_2.d);

			if( v_dqs_s_star_db_2.q > v_q_remnant_2)
			{
				v_dqs_s_star_db_2.q = v_q_remnant_2;
			}
			if( v_dqs_s_star_db_2.q < -v_q_remnant_2)
			{
				v_dqs_s_star_db_2.q = -v_q_remnant_2;
			}*/
		//////////////////////////////////////////////////////////////////

	#endif
	// end _DBDTFC_2
	
	
	#ifdef _DBCR_2
		//iqe_2 = tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2;

			
		i_dqs_e_star_2.q = 0.0;//chirp_q;//+0.25f*cos(two_pi*count*dt*100.0f);//tem_star_2*two_thirds*(1.0f/pole_pairs)/lambda_hat_2;//0.1f;//
		i_dqs_e_star_2.d = 0.0;//chirp_d;//0.0f;//
		// Generate Error
		i_dqs_e_err_2.q = i_dqs_e_star_2.q - i_dqs_e_hat_kp1_2.q;//i_dqs_e_2.q;//
		i_dqs_e_err_2.d = i_dqs_e_star_2.d - i_dqs_e_hat_kp1_2.d;//i_dqs_e_2.d;//

		// Algorithm q-axis
		v_dqs_e_star_dbcr_2.q  	    = v_dqs_e_star_dbcr_2.q;
		v_dqs_e_star_dbcr_int_2.q  += K_dbcr_2*(i_dqs_e_err_2.q*(cosf(pole_pairs*omega_encoder*dt) - C_tau_2) +  i_dqs_e_err_2.d*sinf(pole_pairs*omega_encoder*dt));
		v_dqs_e_star_dbcr_2.q	    = v_dqs_e_star_dbcr_int_2.q + i_dqs_e_err_2.q*C_tau_2*K_dbcr_2 + dist_q;
		// BEMF decoupling
		v_dqs_e_star_dbcr_2.q += lambda_hat_2*pole_pairs*omega_encoder;// + v_dqs_current_obsr_harm_decoup_2.q;

		// Algorithm d-axis
		v_dqs_e_star_dbcr_2.d  	   = v_dqs_e_star_dbcr_2.d + dist_d;
		v_dqs_e_star_dbcr_int_2.d += K_dbcr_2*(i_dqs_e_err_2.d*(cosf(pole_pairs*omega_encoder*dt) - C_tau_2) -  i_dqs_e_err_2.q*sinf(pole_pairs*omega_encoder*dt));
		v_dqs_e_star_dbcr_2.d	   = v_dqs_e_star_dbcr_int_2.d + i_dqs_e_err_2.d*C_tau_2*K_dbcr_2;// + v_dqs_current_obsr_harm_decoup_2.d;C_tau_2
		
		v_dqs_s_star_dbcr_2 = rotate(v_dqs_e_star_dbcr_2, theta_r_comp_2);//+1.8f*pole_pairs*delta_theta/delta_theta_mult);//);
		
		//v_dqs_e_star_dbcr_2_km1.q = v_dqs_e_star_dbcr_2.q;
		//v_dqs_e_star_dbcr_2_km1.d = v_dqs_e_star_dbcr_2.d;
	#endif
	// end _DBCR
	
		// Torque Slewing
	
	/*	if((tem_star_2 - tem_star_pre_2) > tem_star_slew_2){
			tem_star_2 = tem_star_pre_2 + tem_star_slew_2;       
		}
	
		if((tem_star_2 - tem_star_pre_2) < (-tem_star_slew_2)){
		 	tem_star_2 = tem_star_pre_2 - tem_star_slew_2;   
		}
		//Saves the calculated value of torque
		tem_star_pre_2 = tem_star_2;*/
		//	v_dqs_s_star_2.q = 0.0f;
		//	v_dqs_s_star_2.d = 0.0f;


		
	}// end if(drive_flag == RUN)
		
			
				

	if(drive_flag == RUN || drive_flag == STARTUP)
	{
			
		if(torq_mod_select_1 == 1.0f ){
			v_dqs_s_star_1.q = v_dqs_s_star_foc_1.q;
			v_dqs_s_star_1.d = v_dqs_s_star_foc_1.d;
		}
	
	
		if(torq_mod_select_1 == 3.0f ){
			v_dqs_s_star_1.q = v_dqs_s_star_vs_1.q;
			v_dqs_s_star_1.d = v_dqs_s_star_vs_1.d;
		}
		
			
		if(torq_mod_select == 1.0f ){
			v_dqs_s_star_2.q = v_dqs_s_star_foc_2.q;
			v_dqs_s_star_2.d = v_dqs_s_star_foc_2.d;
		}
	
		if(torq_mod_select == 2.0f ){
			v_dqs_s_star_2.q = v_dqs_s_star_db_2.q;
			v_dqs_s_star_2.d = v_dqs_s_star_db_2.d;
		}
	
		if(torq_mod_select == 3.0f ){
			v_dqs_s_star_2.q = v_dqs_s_star_vs_2.q;
			v_dqs_s_star_2.d = v_dqs_s_star_vs_2.d;
		}
		
		if(torq_mod_select == 4.0f ){
			v_dqs_s_star_2.q = v_dqs_s_star_dbcr_2.q;
			v_dqs_s_star_2.d = v_dqs_s_star_dbcr_2.d;
		}
	

	
		// If not using FOC camp current reg integrators from windup
		if(torq_mod_select != 1.0f ){
			i_dqs_e_err_int_2.q = 0.0f;
		    i_dqs_e_err_int_2.d = 0.0f;
		}
		
		
		/**************************************************************************/
		/**************************************************************************/
		/**     Overmodulation / Voltage Limiting								 **/
		/**	 																	 **/
		/**	 	This option allows for dymamic scalling of the voltage           **/
		/**     based on the available inverter voltage.                         **/
		/**************************************************************************/
		/**************************************************************************/
		
		//This voltage limiting case scales back the commanded voltage vector to the largest circle that fits within the inverter voltage limits
	
		
		// Assign Phase Voltage Vectors
		v_a_1 = v_dqs_s_star_1.q;
		v_b_1 = -0.5f*v_dqs_s_star_1.q-sqrt3_over_2*v_dqs_s_star_1.d;
		v_c_1 = -0.5f*v_dqs_s_star_1.q+sqrt3_over_2*v_dqs_s_star_1.d;
		
		v_a_2 = v_dqs_s_star_2.q;//10*sinf(two_pi*500*count*dt);//
		v_b_2 = -0.5f*v_dqs_s_star_2.q-sqrt3_over_2*v_dqs_s_star_2.d;
		v_c_2 = -0.5f*v_dqs_s_star_2.q+sqrt3_over_2*v_dqs_s_star_2.d;
					
		
		/**************************************************************************/
		/**************************************************************************/
		/**     Lagrange Multiplier Loss Minimization Implementation			 **/
		/**	 	Added by Brian Bradley											 **/
		/**	 			computed in stationary ref frame						 **/
		/**	 			transfered to synch frame at end						 **/
		/**	 																	 **/
		/**	 																	 **/
		/**	 																	 **/
		/**************************************************************************/
		/**************************************************************************/		
			
 		
		
			

	
		/**************************************************************************/
		/**************************************************************************/
		/**     Assign voltage out to PWM Duty Cycle							 **/
		/**	 																	 **/
		/**	 																	 **/
		/**************************************************************************/
		/**************************************************************************/
		// Output Voltage Vectors

				//v_a_pwm_1=0.5f+(v_a_1/680);;
		//v_b_pwm_1=0.5f+(v_b_1/680);
		//v_c_pwm_1=0.5f+(v_c_1/680);
	
	
		v_a_pwm_1=0.5f+(v_a_1/v_dc_link);
		v_b_pwm_1=0.5f+(v_b_1/v_dc_link);
	    v_c_pwm_1=0.5f+(v_c_1/v_dc_link);
    
    
		//v_a_pwm_2= 0.5f+((v_a_2-v_n_2)/v_dc_link);
		//v_b_pwm_2= 0.5f+((v_b_2-v_n_2)/v_dc_link);
		//v_c_pwm_2= 0.5f+((v_c_2-v_n_2)/v_dc_link);
	
		v_a_pwm_2=0.5f+((v_a_2-v_n_2)/v_dc_link);//44.0f);
		v_b_pwm_2=0.5f+((v_b_2-v_n_2)/v_dc_link);//44.0f);
		v_c_pwm_2=0.5f+((v_c_2-v_n_2)/v_dc_link);//44.0f);
		//count += 1;

		//v_a_pwm_2=0.5f+((v_a_2-v_n_2)/v_dc_link_filtered);
		//v_b_pwm_2=0.5f+((v_b_2-v_n_2)/v_dc_link_filtered);
		//v_c_pwm_2=0.5f+((v_c_2-v_n_2)/v_dc_link_filtered);	

		#ifdef _SVM
		//	v_dc_link_filtered = 330.0f;// remove when main circuit is turned on
			v_mag_svm=sqrt(v_dqs_s_star_2.q*v_dqs_s_star_2.q+v_dqs_s_star_2.d*v_dqs_s_star_2.d);
			v_phs_svm=phase_in_two_pi(v_dqs_s_star_2.q,v_dqs_s_star_2.d);
			// phase change to [0, 2pi]
				
				
			// which sector the voltage command is in?
			sector=(int)ceil(v_phs_svm/(pi_over_3));
			
			if(sector==0)
			{
				sector==1;
			}
			
			Tsv_a=v_mag_svm*sqrt3/v_dc_link*cos(v_phs_svm-pi_over_3*sector+pi_over_3)*dt;
			Tsv_b=v_mag_svm*sqrt3/v_dc_link*cos(v_phs_svm-pi_over_3*sector-pi_over_3)*dt;
			
			Tsv_0 = (dt-Tsv_a-Tsv_b)*0.5f;
			Tsv_7 = (dt-Tsv_a-Tsv_b)*0.5f;
			
			switch (sector)
			{
				case 1: //sector 1 [0, pi/3]

					v_a_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_b_svm=(Tsv_b+Tsv_7)/dt;
					v_c_svm=(Tsv_7)/dt;
					break;
				
				case 2: //sector 2 [pi/3, 2pi/3]

					v_b_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_a_svm=(Tsv_a+Tsv_7)/dt;
					v_c_svm=(Tsv_7)/dt;
					break;		
				
				case 3: //sector 3 [2pi/3, pi]

					v_b_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_c_svm=(Tsv_b+Tsv_7)/dt;
					v_a_svm=(Tsv_7)/dt;
					break;
				
				case 4: //sector 4 [pi, 4pi/3]

					v_c_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_b_svm=(Tsv_a+Tsv_7)/dt;
					v_a_svm=(Tsv_7)/dt;
					break;			
			
				case 5: //sector 5 [4pi/3, 5pi/3]

					v_c_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_a_svm=(Tsv_b+Tsv_7)/dt;
					v_b_svm=(Tsv_7)/dt;
					break;
					
				case 6: //sector 6 [0, 6pi/3]

					v_a_svm=(Tsv_a+Tsv_b+Tsv_7)/dt;
					v_c_svm=(Tsv_a+Tsv_7)/dt;
					v_b_svm=(Tsv_7)/dt;
					break;		
					
					}
			
			
			v_a_pwm_2=v_a_svm;
			v_b_pwm_2=v_b_svm;
			v_c_pwm_2=v_c_svm;
		#endif
		// end _SVM
	

	
	
	
		/**************************************************************************/
		/**************************************************************************/
		/**     PWM Deadtime comensation										 **/
		/**	 																	 **/
		/**	 	Compensate for deadtime in inverter hardware					 **/
		/**************************************************************************/
		/**************************************************************************/
	
	
		#ifdef _INV_COMP_1
			//Phase A_1
			pwm_t_phase_comp_ref = (pwm_t_comp*i_a_1*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp   = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp : ((pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref);
			v_a_pwm_1+=pwm_t_phase_comp;
			//Phase B_1
			pwm_t_phase_comp_ref = (pwm_t_comp*i_b_1*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp :(pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref;
			v_b_pwm_1+=pwm_t_phase_comp;
			//Phase C_1
			pwm_t_phase_comp_ref = (pwm_t_comp*i_c_1*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp :(pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref;
			v_c_pwm_1+=pwm_t_phase_comp;
		#endif
	
	
		#ifdef _INV_COMP_2
			//Phase A_2
			pwm_t_phase_comp_ref = (pwm_t_comp*i_a_2*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp : ((pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref);
			v_a_pwm_2+=pwm_t_phase_comp;
			//Phase B_2
			pwm_t_phase_comp_ref = (pwm_t_comp*i_b_2*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp :(pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref;
			v_b_pwm_2+=pwm_t_phase_comp;
			//Phase C_2
			pwm_t_phase_comp_ref = (pwm_t_comp*i_c_2*pwm_inv_comp); // Reference dead time value 
			pwm_t_phase_comp = (pwm_t_phase_comp_ref > pwm_t_comp) ? pwm_t_comp :(pwm_t_phase_comp_ref < -pwm_t_comp)? -pwm_t_comp : pwm_t_phase_comp_ref;
			v_c_pwm_2+=pwm_t_phase_comp;
		#endif




		/**************************************************************************/
		/**************************************************************************/
		/**     PWM Output														 **/
		/**	 																	 **/
		/**	 	Saturate and output PWM signals									 **/
		/**************************************************************************/
		/**************************************************************************/
		v_a_pwm_1 = lib_clip(v_a_pwm_1, 0.0f, 1.0f);
		v_b_pwm_1 = lib_clip(v_b_pwm_1, 0.0f, 1.0f);
		v_c_pwm_1 = lib_clip(v_c_pwm_1, 0.0f, 1.0f);

		v_a_pwm_2 = lib_clip(v_a_pwm_2, 0.0f, 1.0f); //0.3f, 0.7f
		v_b_pwm_2 = lib_clip(v_b_pwm_2, 0.0f, 1.0f);
		v_c_pwm_2 = lib_clip(v_c_pwm_2, 0.0f, 1.0f);


		pwm_set_combined_dutycycle(PWM_C.PAIR0, v_a_pwm_1);
		pwm_set_combined_dutycycle(PWM_C.PAIR1, v_b_pwm_1);
		pwm_set_combined_dutycycle(PWM_C.PAIR2, v_c_pwm_1);

		//if(count_PWM == PWM_frequency_divisor)
		//{
			pwm_set_combined_dutycycle(PWM_B.PAIR0, v_a_pwm_2);//v_a_pwm_2
			pwm_set_combined_dutycycle(PWM_B.PAIR1, v_b_pwm_2);//v_b_pwm_2
			pwm_set_combined_dutycycle(PWM_B.PAIR2, v_c_pwm_2);//v_c_pwm_2
			//count_PWM = 0;
		//}
		//count_PWM++;
		
		//i_dqs_s_2_km1.q = i_dqs_s_2.q;
		//i_dqs_s_2_km1.d = i_dqs_s_2.d;

	} // end if(drive_flag == RUN || drive_flag == STARTUP)

	
	
	
	/**************************************************************************/
	/**************************************************************************/
	/**     SS Data Collect 												 **/
	/**	 																	 **/
	/**	 	Collects Steady State operating point data for analysis			 **/
	/**************************************************************************/
	/**************************************************************************/

	#ifdef _SS_DATA_COLLECT   
	
		p_in_1  = 1.5f*(v_dqs_s_star_1.q*i_dqs_s_1.q+v_dqs_s_star_1.d*i_dqs_s_1.d);
		p_in_filt_1  = (1-eff_filt_tau)*p_in_filt_1+eff_filt_tau*p_in_1;
	
		p_out_1 = omega_l_hat_kp1_1*tem_star_1;
		p_out_filt_1 = (1-eff_filt_tau)*p_out_filt_1+eff_filt_tau*p_out_1;
	
		eff_1 	= p_out_1 / p_in_1;
		eff_filt_1   = (1-eff_filt_tau)*eff_filt_1+eff_filt_tau*eff_1;

		p_in_2 	= 1.5f*(v_dqs_s_star_2.q*i_dqs_s_2.q+v_dqs_s_star_2.d*i_dqs_s_2.d);
		p_in_filt_2  = (1-eff_filt_tau)*p_in_filt_2+eff_filt_tau*p_in_2;
	
		p_out_2 = omega_l_hat_kp1_1*tem_star_2;
		p_out_filt_2 = (1-eff_filt_tau)*p_out_filt_2+eff_filt_tau*p_out_2;
	
		eff_2 	= p_out_2 / p_in_2;
		eff_filt_2   = (1-eff_filt_tau)*eff_filt_2+eff_filt_tau*eff_2;
	
		i_s_mag_2 = sqrt(i_dqs_s_star_2.d*i_dqs_s_star_2.d+i_dqs_s_star_2.q*i_dqs_s_star_2.q);
		i_s_mag_filt_2 = (1-eff_filt_tau)*i_s_mag_filt_2+eff_filt_tau*i_s_mag_2;

	
		v_s_mag_2 = sqrt(v_dqs_s_star_2.q*v_dqs_s_star_2.q+v_dqs_s_star_2.d*v_dqs_s_star_2.d);
		v_s_mag_filt_2 = (1-eff_filt_tau)*v_s_mag_filt_2+eff_filt_tau*v_s_mag_2;
	
		lambda_dqs_e_filt_2_d =  (1-eff_filt_tau)*lambda_dqs_e_filt_2_d+eff_filt_tau*lambda_dqs_e_2.d;
	
		omega_l_hat_kp1_filt_1 = (1-eff_filt_tau)*omega_l_hat_kp1_filt_1+eff_filt_tau*omega_l_hat_kp1_1;
	
	#endif

	

	
	
	#ifdef _DATA_LOGGING
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Data Logging													 **/
	/**	 																	 **/
	/**	 	Log off variables for use in XCS data linker					 **/
	/**	 																	 **/
	/**	 																	 **/
	/**	 	rec_flag = 	0 --> Ready to Record								 **/
	/**					1 --> Recording 									 **/
	/**	 				2 --> Done Recording								 **/
	/**	 				3 --> Reset											 **/
	/**	 																	 **/
	/**	 																	 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/
	
	
 		// Only log if record flag is active and dump count is less 
  		//than the number of samples being recorded
	    if(flag_rec == 1 && (log_dump_count < log_num_samples))
	    
	    {
		    // Save values at decimated steps
	        if(log_decimate_count >= log_sample_decimate) {  //0209 if(log_decimate_count == 1){

	           if (dataset == 1.0f){



	            //value_dump[log_dump_count][0]   = i_dqs_e_star_2.q;//lambda_dqr_r_cm_2.q; //v_dqs_s_star_2.q;
				//value_dump[log_dump_count][1]   = i_dqs_e_star_2.d; //v_dqs_s_star_2.d;
				value_dump[log_dump_count][0]   = E_norm_d_s_A;//iqe_2;//lambda_dqr_r_cm_2.q;//chirp_q;//i_dqs_s_2.q;
				value_dump[log_dump_count][1]   = E_norm_q_s_A;//count;//lambda_dqr_r_cm_2.d;//chirp_d;//i_dqs_s_2.d;
				value_dump[log_dump_count][2]   = omega_encoder_1;//v_dc_link;
				value_dump[log_dump_count][3]   = theta;//omega_l_hat_kp1_1;
				value_dump[log_dump_count][4]   = v_a_2_measured;//v_ab_2_ad;//i_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][5]   = v_b_2_measured;//v_ca_2_ad;//i_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][6]   = v_c_2_measured;//lambda_dqs_s_hat_kp1_2.q ;
				value_dump[log_dump_count][7]   = v_a_2;//i_dqs_e_2.d;//omega_enc_filtered;//omega_r_hat_kp1_1;//lambda_dqs_s_hat_kp1_2.d ;
				value_dump[log_dump_count][8]  = v_b_2;//omega_encoder_1;//omega_encoder;//lambda_dqr_s_hat_kp1_2.q;
				value_dump[log_dump_count][9]  = v_c_2;//v_ab_2_ad;//ambda_dqr_s_hat_kp1_2.d;
				value_dump[log_dump_count][10]  = E_norm_d_s_M;//v_bc_2_ad;//i_dqs_e_2.q;
				value_dump[log_dump_count][11]  = E_norm_q_s_M;//v_ca_2_ad;
				//value_dump[log_dump_count][10]  = v_dc_link;
				//value_dump[log_dump_count][11]  = enc_cnt;////v_dqs_e_star_db_2.q;//tem_hat_kp1_2;
				//value_dump[log_dump_count][12]  = theta;//v_dqs_e_star_db_2.d;
				//value_dump[log_dump_count][13]  = delta_theta_1;//dist_q;//v_dqs_e_star_foc_2.q;
				//value_dump[log_dump_count][14]  = delta_theta_mult;//dist_d;//v_dqs_e_star_foc_2.d;
				//value_dump[log_dump_count][15]  = pwm_frequency;//tem_star_2;
				//value_dump[log_dump_count][16]  = tem_hat_kp1_2;

		
				
				/*value_dump[log_dump_count][20]  = w_bar;
				value_dump[log_dump_count][21]  = 0.0f;
				value_dump[log_dump_count][22]  = 0.0f;
				value_dump[log_dump_count][23]  = 0.0f;
				value_dump[log_dump_count][24]  = 0.0f;
				value_dump[log_dump_count][25]  = 0.0f;
				value_dump[log_dump_count][26]  = 0.0f;
				value_dump[log_dump_count][27]  = 0.0f;
				value_dump[log_dump_count][28]  = 0.0f;
				value_dump[log_dump_count][29]  = 0.0f;*/
				
	            }
				
				
				if (dataset == 2.0f){
				/*    
				value_dump[log_dump_count][0]   = v_dc_link_filtered; //i_dqs_s_hat_2.q;//;//i_a_1
				value_dump[log_dump_count][1]   = v_dc_link; //i_dqs_s_hat_2.d;//v_dc_link;
				value_dump[log_dump_count][2]   = i_c_1; //v_dqs_s_star_2.q;
				value_dump[log_dump_count][3]   = i_a_2; //v_dqs_s_star_2.d;
				value_dump[log_dump_count][4]   = i_b_2; //i_dqs_s_2.q;
				value_dump[log_dump_count][5]   = i_c_2; //i_dqs_s_2.d;
				value_dump[log_dump_count][6]   = v_dqs_s_star_2.q; //lambda_dqr_s_cm_2.q;//v_dqs_s_star_1.q;
				value_dump[log_dump_count][7]   = v_dqs_s_star_2.d; //lambda_dqr_s_cm_2.d;//v_dqs_s_star_1.q;
				value_dump[log_dump_count][8]   = v_dqs_s_star_1.q; //lambda_dqr_s_vm_2.q;//i_dqs_s_1.q;
				value_dump[log_dump_count][9]   = v_dqs_s_star_1.d; //lambda_dqr_s_vm_2.d;//i_dqs_s_1.d;
				value_dump[log_dump_count][10]  = v_a_pwm_1; //i_dqr_s_2.q;
				value_dump[log_dump_count][11]  = v_b_pwm_1; //i_dqr_s_2.d;
				value_dump[log_dump_count][12]  = v_c_pwm_1; //i_c_1; //lambda_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][13]  = v_a_pwm_2; //lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][14]  = v_b_pwm_2; //lambda_dqr_s_hat_kp1_2.q;
				value_dump[log_dump_count][15]  = v_c_pwm_2; //lambda_dqr_s_hat_kp1_2.d;
				value_dump[log_dump_count][16]  = v_dqs_s_star_foc_1.q; //tem_hat_kp1_2;
				value_dump[log_dump_count][17]  = v_dqs_s_star_foc_1.d; //p_cond_2;
				value_dump[log_dump_count][18]  = tem_hat_kp1_2;
				value_dump[log_dump_count][19]  = p_loss_2;
				
				
				value_dump[log_dump_count][20]  = lamda_star_1;
				value_dump[log_dump_count][21]  = lambda_s_star_2;   // log stator flux command for DB-DTFC
				value_dump[log_dump_count][22]  = tem_star_1;
				value_dump[log_dump_count][23]  = tem_star_2;
				value_dump[log_dump_count][24]  = tem_crank;
				value_dump[log_dump_count][25]  = theta;
				value_dump[log_dump_count][26]  = omega_encoder;
				value_dump[log_dump_count][27]  = omega_l_hat_k_1;
				value_dump[log_dump_count][28]  = omega_r_hat_k_1;
				value_dump[log_dump_count][29]  = theta_err_1;
				*/
				/*
				value_dump[log_dump_count][0]   = omega_enc_filtered;
				value_dump[log_dump_count][1]   = omega_encoder;
				value_dump[log_dump_count][2]  = omega_l_hat_k_1;
				value_dump[log_dump_count][3]  = w_bar;
				value_dump[log_dump_count][4]  = chirp;
				*/

				}
				
				/*			
				if (dataset == 3.0f){																	//Matlab
				value_dump[log_dump_count][0]   = v_dqs_s_star_2.q;//chirp;//v_dqs_s_star_2.q;
				value_dump[log_dump_count][1]   = v_dqs_s_star_2.d;//omega_hat_kp1_1;//omega_r_hat_kp1_1;//v_dqs_s_star_2.d;
				value_dump[log_dump_count][2]   = i_dqs_s_2.q;
				value_dump[log_dump_count][3]   = i_dqs_s_2.d;
				value_dump[log_dump_count][4]   = tem_star_avg ;//COST[0];//tem_star_ovm_2;//drivecycle_omega_dot_2;//omega_dot_filt_kp1_1;//tem_star_ovm_2; //test_5;//tem.k1; //tem_hat_om_kp1_2;//omega_r_hat_kp1_1;
				value_dump[log_dump_count][5]   = omega_l_hat_kp1_1;
				value_dump[log_dump_count][6]   = tem_star_1;//lambda_s_star_mtpa;//b_2_pre;//tem_star_1;
				value_dump[log_dump_count][7]   = tem_star_2;
				value_dump[log_dump_count][8]   = w_bar;//alpha;//omega_hat_k_1;//index_min;
				value_dump[log_dump_count][9]	= tem_star_sfb_total_2;//Pt1_current;
				value_dump[log_dump_count][10]  = lambda_s_star_ovm_2;
				value_dump[log_dump_count][11]  = lambda_s_star_2;//lambda_s_star_cmd_2;
				value_dump[log_dump_count][12]  = tem_hat_kp1_2;
				value_dump[log_dump_count][13]  = delta_theta;//lambda_dqs_e_2.d;//Pt2_current;
				value_dump[log_dump_count][14]  = omega_encoder;//v_dc_link_filtered;
				value_dump[log_dump_count][15]  = omega_star_kp1_2;
				value_dump[log_dump_count][16]  = tem_crank;//Pt3_current;//tem_star_ovm_slew_2;//v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][17]  = tem_star_ovm_2;//v_dqs_s_star_pre_2.d;//Pt4_current;//
				value_dump[log_dump_count][18]  = v_dc_link;//m_2;
				value_dump[log_dump_count][19]  = b_2;
				value_dump[log_dump_count][20]  = lambda_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][21]  = lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][22]  = lambda_dqr_s_hat_kp1_2.q;//Pt5_current;//
				value_dump[log_dump_count][23]  = lambda_dqr_s_hat_kp1_2.d;//alpha_A;//
				value_dump[log_dump_count][24]  = p_out_2;//i_dqs_es_2.q;//i_dqs_s_hat_kp1_2.q;//COST[6];//
				value_dump[log_dump_count][25]  = p_in_2;//delta_theta_err_2;//omega_r_hat_kp1_1;//i_dqs_es_2.d;//i_dqs_s_hat_kp1_2.d;
				//value_dump[log_dump_count][26]  = rpm;//v_dqs_s_star_pre_2.q;
				//value_dump[log_dump_count][27]  = i_b_2;//v_dqs_s_star_pre_2.d;
				//value_dump[log_dump_count][28]  = i_c_2;//lambda_dqr_e_2.q;
				//value_dump[log_dump_count][29]  = ENCOD_A.POS_A;//lambda_dqr_e_2.d;
				value_dump[log_dump_count][26]  = i_dqs_e_star_2.q;
				value_dump[log_dump_count][27]  = i_dqs_e_star_2.d;
				value_dump[log_dump_count][28]  = i_dqs_e_2.q;
				value_dump[log_dump_count][29]  = i_dqs_e_2.d;
				
			
				// this is from before starting the COST changes (used for digest)
				value_dump[log_dump_count][0]   = v_dqs_s_star_2.q;//chirp;//v_dqs_s_star_2.q;
				value_dump[log_dump_count][1]   = v_dqs_s_star_2.d;//omega_hat_kp1_1;//omega_r_hat_kp1_1;//v_dqs_s_star_2.d;
				value_dump[log_dump_count][2]   = i_dqs_s_2.q;
				value_dump[log_dump_count][3]   = i_dqs_s_2.d;
				value_dump[log_dump_count][4]   = torq_obsr_ctrl_2;//PtA_current;//tem_star_ovm_2;//drivecycle_omega_dot_2;//omega_dot_filt_kp1_1;//tem_star_ovm_2; //test_5;//tem.k1; //tem_hat_om_kp1_2;//omega_r_hat_kp1_1;
				value_dump[log_dump_count][5]   = omega_l_hat_kp1_1;
				value_dump[log_dump_count][6]   = tem_star_1;//lambda_s_star_mtpa;//b_2_pre;//tem_star_1;
				value_dump[log_dump_count][7]   = tem_star_2;
				value_dump[log_dump_count][8]   = omega_hat_k_1;//index_min;
				value_dump[log_dump_count][9]	= tem_star_sfb_total_2;//Pt1_current;
				value_dump[log_dump_count][10]  = lambda_s_star_ovm_2;
				value_dump[log_dump_count][11]  = lambda_s_star_2;//lambda_s_star_cmd_2;
				value_dump[log_dump_count][12]  = tem_hat_kp1_2;
				value_dump[log_dump_count][13]  = theta_hat_kp1_1;//Pt2_current;//lambda_dqs_e_2.d;
				value_dump[log_dump_count][14]  = v_dc_link_filtered;
				value_dump[log_dump_count][15]  = omega_star_kp1_2;
				value_dump[log_dump_count][16]  = delta_theta;//chirp;//over_mod_flag;//Pt3_current;//tem_star_ovm_slew_2;//v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][17]  = torq_obsr_2a;//delta_theta_hat_k_2;//Pt4_current;//tem_star_ovm_2;//v_dqs_s_star_pre_2.d;
				value_dump[log_dump_count][18]  = m_2;
				value_dump[log_dump_count][19]  = b_2;
				value_dump[log_dump_count][20]  = lambda_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][21]  = lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][22]  = p_loss_2;//delta_theta_err_accum2_2;//Pt5_current;//lambda_dqr_s_hat_kp1_2.q;
				value_dump[log_dump_count][23]  = lambda_dqr_s_hat_kp1_2.d;
				value_dump[log_dump_count][24]  = delta_theta_err_accum_2;//torq_obsr_2;//PtC_current;//i_dqs_es_2.q;//i_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][25]  = p_in_2;//delta_theta_err_2;//omega_r_hat_kp1_1;//i_dqs_es_2.d;//i_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][26]  = v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][27]  = v_dqs_s_star_pre_2.d;
				value_dump[log_dump_count][28]  = lambda_dqr_e_2.q;
				value_dump[log_dump_count][29]  = lambda_dqr_e_2.d;
				
				
				}
				
				
					if (dataset == 4.0f){																	//Matlab
			    value_dump[log_dump_count][0]   = v_dqs_s_star_2.q;//chirp;//v_dqs_s_star_2.q;
				value_dump[log_dump_count][1]   = v_dqs_s_star_2.d;//omega_hat_kp1_1;//omega_r_hat_kp1_1;//v_dqs_s_star_2.d;
				value_dump[log_dump_count][2]   = i_dqs_s_2.q;
				value_dump[log_dump_count][3]   = i_dqs_s_2.d;
				value_dump[log_dump_count][4]   = tem_star_avg ;//COST[0];//tem_star_ovm_2;//drivecycle_omega_dot_2;//omega_dot_filt_kp1_1;//tem_star_ovm_2; //test_5;//tem.k1; //tem_hat_om_kp1_2;//omega_r_hat_kp1_1;
				value_dump[log_dump_count][5]   = omega_l_hat_kp1_1;
				value_dump[log_dump_count][6]   = tem_star_1;//lambda_s_star_mtpa;//b_2_pre;//tem_star_1;
				value_dump[log_dump_count][7]   = tem_star_2;
				value_dump[log_dump_count][8]   = w_bar;//alpha;//omega_hat_k_1;//index_min;
				value_dump[log_dump_count][9]	= tem_star_sfb_total_2;//Pt1_current;
				value_dump[log_dump_count][10]  = lambda_s_star_ovm_2;
				value_dump[log_dump_count][11]  = lambda_s_star_2;//lambda_s_star_cmd_2;
				value_dump[log_dump_count][12]  = tem_hat_kp1_2;
				value_dump[log_dump_count][13]  = lambda_dqs_e_2.d;//Pt2_current;
				value_dump[log_dump_count][14]  = omega_encoder;//v_dc_link_filtered;
				value_dump[log_dump_count][15]  = omega_star_kp1_2;
				value_dump[log_dump_count][16]  = i_dqs_e_star_1.q;
				value_dump[log_dump_count][17]  = i_dqs_e_star_1.d;
				value_dump[log_dump_count][18]  = i_dqs_e_1.q;
				value_dump[log_dump_count][19]  = i_dqs_e_1.d;
				value_dump[log_dump_count][20]  = lambda_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][21]  = lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][22]  = lambda_dqr_s_hat_kp1_2.q;//Pt5_current;//
				value_dump[log_dump_count][23]  = lambda_dqr_s_hat_kp1_2.d;//alpha_A;//
				value_dump[log_dump_count][24]  = p_out_2;//i_dqs_es_2.q;//i_dqs_s_hat_kp1_2.q;//COST[6];//
				value_dump[log_dump_count][25]  = p_in_2;//delta_theta_err_2;//omega_r_hat_kp1_1;//i_dqs_es_2.d;//i_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][26]  = i_dqs_e_star_2.q;
				value_dump[log_dump_count][27]  = i_dqs_e_star_2.d;
				value_dump[log_dump_count][28]  = i_dqs_e_2.q;
				value_dump[log_dump_count][29]  = i_dqs_e_2.d;

				
				
				
				}
				
				if (dataset == 4.0f){
				    
				value_dump[log_dump_count][0]   = enc_cnt;
				value_dump[log_dump_count][1]   = delta_cnt;
				value_dump[log_dump_count][2]   = theta;
				value_dump[log_dump_count][3]   = delta_theta;
				value_dump[log_dump_count][4]   = omega_encoder;//t_cff_obsr_1;
				value_dump[log_dump_count][5]   = w_bar;//theta_err_accum_1*kiso_1+theta_err_1*kso_1;
				value_dump[log_dump_count][6]   = rpm;
				value_dump[log_dump_count][7]   = omega_l_hat_k_1;
				value_dump[log_dump_count][8]   = omega_r_hat_k_1;//tem_star_1;
				value_dump[log_dump_count][9]   = theta_hat_k_1;//tem_star_2;
				value_dump[log_dump_count][10]  = omega_dot_kp1_1;
				value_dump[log_dump_count][11]  = omega_dot_filt_kp1_1;//theta_err_accum_1;
				value_dump[log_dump_count][12]  = theta_err_1;
				value_dump[log_dump_count][13]  = omega_star_kp1_2;//t_cff_sfb_2;
				
				
				value_dump[log_dump_count][14]  = 0.0f;
				value_dump[log_dump_count][15]  = 0.0f;
				value_dump[log_dump_count][16]  = 0.0f;
				value_dump[log_dump_count][17]  = 0.0f;
				value_dump[log_dump_count][18]  = 0.0f;
				value_dump[log_dump_count][19]  = 0.0f;
				value_dump[log_dump_count][20]  = 0.0f;
				value_dump[log_dump_count][21]  = 0.0f;
				value_dump[log_dump_count][22]  = 0.0f;
				value_dump[log_dump_count][23]  = 0.0f;
				value_dump[log_dump_count][24]  = 0.0f;
				value_dump[log_dump_count][25]  = 0.0f;
				value_dump[log_dump_count][26]  = 0.0f;
				value_dump[log_dump_count][27]  = 0.0f;
				value_dump[log_dump_count][28]  = 0.0f;
				value_dump[log_dump_count][29]  = 0.0f;
				
				}

				
				if (dataset == 5.0f){
				value_dump[log_dump_count][0]   = v_dqs_s_star_2.q;
				value_dump[log_dump_count][1]   = v_dqs_s_star_2.d;
				value_dump[log_dump_count][2]   = delta_tem_2;
				value_dump[log_dump_count][3]   = test_5;
				value_dump[log_dump_count][4]   = omega_l_hat_kp1_1;
				value_dump[log_dump_count][5]   = tem_star_2;
				value_dump[log_dump_count][6]   = lambda_s_star_2;
				value_dump[log_dump_count][7]   = tem_hat_kp1_2;
				value_dump[log_dump_count][8]   = lambda_dqs_e_2.d;
				value_dump[log_dump_count][9]   = lambda_dqr_e_2.q;
				value_dump[log_dump_count][10]  = lambda_dqr_e_2.d;
				value_dump[log_dump_count][11]  = v_dc_link_filtered;
				value_dump[log_dump_count][12]  = v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][13]  = v_dqs_s_star_pre_2.d;
				value_dump[log_dump_count][14]  = m_2;
				value_dump[log_dump_count][15]  = b_2;
				value_dump[log_dump_count][16]  = lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][17]  = lambda_dqs_s_hat_kp1_2.q;
				
				
				value_dump[log_dump_count][18]  = 0.0f;
				value_dump[log_dump_count][19]  = 0.0f;
				value_dump[log_dump_count][20]  = 0.0f;
				value_dump[log_dump_count][21]  = 0.0f;
				value_dump[log_dump_count][22]  = 0.0f;
				value_dump[log_dump_count][23]  = 0.0f;
				value_dump[log_dump_count][24]  = 0.0f;
				value_dump[log_dump_count][25]  = 0.0f;
				value_dump[log_dump_count][26]  = 0.0f;
				value_dump[log_dump_count][27]  = 0.0f;
				value_dump[log_dump_count][28]  = 0.0f;
				value_dump[log_dump_count][29]  = 0.0f;
				

				}
				
				// Dataset added by Brian
				if (dataset == 6.0f){
				value_dump[log_dump_count][0]   = v_dqs_s_star_2.q;
				value_dump[log_dump_count][1]   = v_dqs_s_star_2.d;
				value_dump[log_dump_count][2]   = i_dqs_s_2.q;
				value_dump[log_dump_count][3]   = i_dqs_s_2.d;
				value_dump[log_dump_count][4]   = lambda_dqs_e_opt_kp1_2.d; //tem.k1; //tem_hat_om_kp1_2;//omega_r_hat_kp1_1;
				value_dump[log_dump_count][5]   = omega_l_hat_kp1_1;
				value_dump[log_dump_count][6]   = tem_star_1;
				value_dump[log_dump_count][7]   = tem_star_2;
				value_dump[log_dump_count][10]  = lambda_s_star_2;
				value_dump[log_dump_count][11]  = lambda_s_star_cmd_2;
				value_dump[log_dump_count][12]  = tem_hat_kp1_2;
				value_dump[log_dump_count][13]  = lambda_dqs_e_2.d;
				value_dump[log_dump_count][14]  = v_dc_link_filtered;
				value_dump[log_dump_count][15]  = omega_star_kp1_2;
				value_dump[log_dump_count][16]  = v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][17]  = v_dqs_s_star_pre_2.d;
				value_dump[log_dump_count][18]  = m_2;
				value_dump[log_dump_count][19]  = b_2;
				value_dump[log_dump_count][20]  = lambda_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][21]  = lambda_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][22]  = lambda_dqr_s_hat_kp1_2.q;
				value_dump[log_dump_count][23]  = lambda_dqr_s_hat_kp1_2.d;
				value_dump[log_dump_count][24]  = i_dqs_s_hat_kp1_2.q;
				value_dump[log_dump_count][25]  = i_dqs_s_hat_kp1_2.d;
				value_dump[log_dump_count][26]  = v_dqs_s_star_pre_2.q;
				value_dump[log_dump_count][27]  = v_dqs_s_star_pre_2.d;
				value_dump[log_dump_count][28]  = lambda_dqr_e_2.q;
				value_dump[log_dump_count][29]  = lambda_dqr_e_2.d;
				

				}*/

	        	// Increment Counters
				log_dump_count++;
				log_data_length = log_dump_count;
			 	log_decimate_count++;
			 	
			 	
			   	if(log_dump_count == log_num_samples){					// Done Recording
					flag_rec == 2;
				}
			}
	        else
	        {
	        	// Wait decimation period
	        	log_decimate_count++;
	        }
	        	        
	        // reset count after waiting long enough
	        if(log_decimate_count > log_sample_decimate){
	        log_decimate_count=1; 
	        }
	        
	    }
	    
	    if(flag_rec == 3 ){
		    log_decimate_count=1;
		    log_dump_count = 0;
		    	
	        flag_rec = 0;
//	        k_c = 0;
	    }
	#endif    
	    
	    

		
	#ifdef _DYN_DATA_COLLECT    	        
	/**************************************************************************/
	/**************************************************************************/
	/**     Dynamic Data Collection Macro									 **/
	/**	 	                                                          		 **/
	/** 	Allow for pretriggering of datalogger during dynamic events		 **/
	/**																		 **/	
	/**                                         	  						 **/
	/**************************************************************************/
	/**************************************************************************/   
	
	if ((drive_flag == RUN)){
	    
	    // Do Nothing if Flag =1 Just a Wait State
	    if(dyn_macro_flag == 1.0f){
	        
	        // Exit State on External Trigger
	        if(dyn_macro_cmd != 0.0f){
	            
	            dyn_macro_flag 	= 2.0f;
	            dyn_macro_cmd  	= 0.0f;
	            dyn_macro_timer = 	dyn_macro_pretrig;
	            flag_rec = 3; // reset data logger
	            //flag_start_chirp = 1.0f;
	            //k_c = 0;
	            if (dyn_load_select_num == 2){
	                tem_crank = t_compressor[0];
	                    
	            }
	            if (dyn_load_select_num == 3){
	                tem_crank = tem_star_sin_offset_1;
	            }
	        }
	    }
	    
	    if(dyn_macro_flag == 2.0f){
	    	

	    	dyn_macro_timer = dyn_macro_timer -1;
	    	
	    	// Decrement Timer till next phase
	    	if(dyn_macro_timer == 0){
	    	dyn_macro_flag = 3.0f;
	    	
	    	
	    	}
	    }
	    
	    	    
	    if(dyn_macro_flag == 3.0f){
	        
	        flag_rec = 1;
	        
	         load_select_1 = dyn_load_select_num;
	         if (dyn_load_select_num == 2){
	             theta_c = 0;
	             wc_dt = 2*pi*Cyclical_freq*dt;
	         }
	         
	         if (dyn_load_select_num == 3){
	             cmd_gen_step = 0;
	         }
	    	
	         k_l = 0;
		
			// Go back to start state
			dyn_macro_flag = 1.0f;
	    }	    
	}
	
	/**************************************************************************/
	/**************************************************************************/
	/**     Torque Offset for Dynamics Macro								 **/
	/**	 						                         					 **/
	/**	 	                     											 **/
	/**************************************************************************/
	/**************************************************************************/    
	    
	
	// Check if greater than slew    
		if((dyn_macro_torque_cmd - dyn_macro_torque_offset) > dyn_macro_torque_slew ){
		    dyn_macro_torque_offset = dyn_macro_torque_offset+dyn_macro_torque_slew;}
	    else{
		    if( (dyn_macro_torque_cmd - dyn_macro_torque_offset)  < (-dyn_macro_torque_slew) ){
	           dyn_macro_torque_offset = dyn_macro_torque_offset-dyn_macro_torque_slew;}
			else{ dyn_macro_torque_offset = dyn_macro_torque_cmd; }
		}

	#endif    
	    
	
	
	
	
	
	    
	#ifdef _SS_DATA_COLLECT    	        
	/**************************************************************************/
	/**************************************************************************/
	/**     SS Data Collection Macro										 **/
	/**	 	When Enabled takes SS data for a particular speed / torque		 **/
	/** 																	 **/
	/**																		 **/	
	/**     Should always enter at rated flux								 **/
	/**************************************************************************/
	/**************************************************************************/   
	
	if ( (ss_collect == 1.0f) && (drive_flag == RUN)){


	    if( ss_collect_flag == 0){
	        ss_collect_omega_star = ss_collect_omega_max;
	        ss_collect_torque_star = ss_collect_torque_min;
	        ss_collect_lambda_star = ss_collect_lambda_rated;
	        ss_collect_dump_count = 0;
	        ss_collect_flag =1;	// Exit to next state
	        ss_collect_timer = 50000;}

	    if( ss_collect_flag == 1){
	        ss_collect_timer -= 1;
	        if(ss_collect_timer == 0){
	        ss_collect_flag =2;} } // Move On
	        
	        //flag_start_chirp = 0.0f;    

	    if( ss_collect_flag == 2){
	        

	        //flag_start_chirp = 1.0f;
	        
	        	value_dump[ss_collect_dump_count][0]   = ss_collect_omega_star;
				value_dump[ss_collect_dump_count][1]   = ss_collect_torque_star;
				value_dump[ss_collect_dump_count][2]   = ss_collect_lambda_star;
				value_dump[ss_collect_dump_count][3]   = omega_l_hat_kp1_filt_1; 
				value_dump[ss_collect_dump_count][4]   = ba_1;
				value_dump[ss_collect_dump_count][5]   = lambda_dqs_e_filt_2_d;
				value_dump[ss_collect_dump_count][6]   = p_in_filt_1;
				value_dump[ss_collect_dump_count][7]   = p_in_filt_2;
				value_dump[ss_collect_dump_count][8]   = p_out_filt_1;
				value_dump[ss_collect_dump_count][9]   = p_out_filt_2;
				value_dump[ss_collect_dump_count][10]  = v_s_mag_filt_2;
				value_dump[ss_collect_dump_count][11]  = i_s_mag_filt_2 ;
				value_dump[ss_collect_dump_count][12]  = i_s_mag_2;
				value_dump[ss_collect_dump_count][13]  = lambda_dqs_e_2.d;
				value_dump[ss_collect_dump_count][14]  = 0;
				value_dump[ss_collect_dump_count][15]  = 0;
				value_dump[ss_collect_dump_count][16]  = 0;
				value_dump[ss_collect_dump_count][17]  = 0;
				value_dump[ss_collect_dump_count][18]  = 0;
				value_dump[ss_collect_dump_count][19]  = 0;
	        	ss_collect_dump_count += 1;
	        	
	        	ss_collect_flag =3; // Break out of Dec Lambda Loop
	        	
	        	if( (ss_collect_lambda_star > ss_collect_lambda_min) && (i_s_mag_filt_2 < ss_collect_is_max) ){
	        	 	ss_collect_lambda_star -= ss_collect_lambda_step;
	        	 	ss_collect_timer = 50000;
	        	    ss_collect_flag =1;}   
	        	}
	     
	        	   	
		if( ss_collect_flag == 3){
			ss_collect_lambda_star = ss_collect_lambda_rated + ss_collect_lambda_step;
			ss_collect_timer = 50000;
	        ss_collect_flag =4; } // Move On  
	        
	    if( ss_collect_flag == 4){
	        ss_collect_timer -= 1;
	        if(ss_collect_timer == 0){
	        ss_collect_flag =5;} } // Move On 
	        
	    if( ss_collect_flag == 5){
	        
	        		        
	        	value_dump[ss_collect_dump_count][0]   = ss_collect_omega_star;
				value_dump[ss_collect_dump_count][1]   = ss_collect_torque_star;
				value_dump[ss_collect_dump_count][2]   = ss_collect_lambda_star;
				value_dump[ss_collect_dump_count][3]   = omega_l_hat_kp1_filt_1; 
				value_dump[ss_collect_dump_count][4]   = ba_1;
				value_dump[ss_collect_dump_count][5]   = lambda_dqs_e_filt_2_d;
				value_dump[ss_collect_dump_count][6]   = p_in_filt_1;
				value_dump[ss_collect_dump_count][7]   = p_in_filt_2;
				value_dump[ss_collect_dump_count][8]   = p_out_filt_1;
				value_dump[ss_collect_dump_count][9]   = p_out_filt_2;
				value_dump[ss_collect_dump_count][10]  = v_s_mag_filt_2;
				value_dump[ss_collect_dump_count][11]  = i_s_mag_filt_2 ;
				value_dump[ss_collect_dump_count][12]  = i_s_mag_2;
				value_dump[ss_collect_dump_count][13]  = lambda_dqs_e_2.d;
				value_dump[ss_collect_dump_count][14]  = 0;
				value_dump[ss_collect_dump_count][15]  = 0;
				value_dump[ss_collect_dump_count][16]  = 0;
				value_dump[ss_collect_dump_count][17]  = 0;
				value_dump[ss_collect_dump_count][18]  = 0;
				value_dump[ss_collect_dump_count][19]  = 0;
	        	ss_collect_dump_count += 1;
	        	
	    
	        	
	        	ss_collect_flag =10; // Break out of Dec Lambda Loop
	        	
	        	
	        	
	        	if( (ss_collect_lambda_star >= ss_collect_lambda_max) | (i_s_mag_filt_2 >= ss_collect_is_max) | (v_s_mag_filt_2 >= ss_collect_vs_max) ){	// Exit Conditions
	        	 	
	        	    	if( ss_collect_torque_star < ss_collect_torque_max ){
	        	    	    ss_collect_flag =6;
	        	    	  }
	        	    

	        	    	if( (ss_collect_torque_star >= ss_collect_torque_max) && (ss_collect_omega_star > ss_collect_omega_min)){
	        	    	    ss_collect_flag =8;
	        	    	}
	        	    	
	        	    	if( (ss_collect_torque_star >= ss_collect_torque_max) && (ss_collect_omega_star <= ss_collect_omega_min)){
	        	    	    ss_collect_flag =10;
	        	    	}
	        	    	  
	        	}

	        	
	        	if( (ss_collect_lambda_star < ss_collect_lambda_max) && (i_s_mag_filt_2 < ss_collect_is_max) && (v_s_mag_filt_2 < ss_collect_vs_max) ){
	        	 	ss_collect_lambda_star += ss_collect_lambda_step;
	        	 	ss_collect_timer = 50000;
	        	    ss_collect_flag =4;}
	        	    
	        	       
	    }
	    
	    
	    
	    if( ss_collect_flag == 6){
			ss_collect_lambda_star = ss_collect_lambda_rated;
			ss_collect_timer = 50000;
	        ss_collect_flag =7; } // Move On  
	        
	    if( ss_collect_flag == 7){
	        ss_collect_timer -= 1;
	        if(ss_collect_timer == 0){
	        ss_collect_torque_star += ss_collect_torque_step;
	        ss_collect_timer = 50000;
	        ss_collect_flag =1;} } // Move On 
	        
	        
	    if( ss_collect_flag == 8){
			ss_collect_lambda_star = ss_collect_lambda_rated;
			ss_collect_torque_star = ss_collect_torque_min;
			ss_collect_timer = 50000;
	        ss_collect_flag =9; } // Move On  
	        
	    if( ss_collect_flag == 9){
	        ss_collect_timer -= 1;
	        if(ss_collect_timer == 0){
	        ss_collect_omega_star -= ss_collect_omega_step;
	        ss_collect_timer = 50000;
	        ss_collect_flag =1;} } // Move On 
	        
	        
	    if( ss_collect_flag == 10){
			ss_collect_lambda_star = ss_collect_lambda_rated;
			ss_collect_torque_star = ss_collect_torque_min;
			ss_collect_timer = 50000;
	        ss_collect_flag =11; } // Move On 
	        
	        
	    if( ss_collect_flag == 11){
	        ss_collect_timer -= 1;
	        if(ss_collect_timer == 0){
	        ss_collect_omega_star = ss_collect_omega_max;
	        ss_collect_timer = 50000;
	        ss_collect_flag =12;} } // Move On  
	        

	     // Update System with new parameters   
		 omega_star_const_2 = ss_collect_omega_star;
	     ba_1 = ss_collect_torque_star / ss_collect_omega_max;
	     lambda_s_star_cmd_2 = ss_collect_lambda_star;
	    

	    
	    
	}
	
	if (ss_collect == 2.0f){
	 
	 ss_collect_flag = 0;
	 
	 
	 
	 			ss_collect_dump_count -= 1;
	 			value_dump[ss_collect_dump_count][0]   = 0;
				value_dump[ss_collect_dump_count][1]   = 0;
				value_dump[ss_collect_dump_count][2]   = 0;
				value_dump[ss_collect_dump_count][3]   = 0; 
				value_dump[ss_collect_dump_count][4]   = 0;
				value_dump[ss_collect_dump_count][5]   = 0;
				value_dump[ss_collect_dump_count][6]   = 0;
				value_dump[ss_collect_dump_count][7]   = 0;
				value_dump[ss_collect_dump_count][8]   = 0;
				value_dump[ss_collect_dump_count][9]   = 0;
				value_dump[ss_collect_dump_count][10]  = 0;
				value_dump[ss_collect_dump_count][11]  = 0;
				value_dump[ss_collect_dump_count][12]  = 0;
				value_dump[ss_collect_dump_count][13]  = 0;
				value_dump[ss_collect_dump_count][14]  = 0;
				value_dump[ss_collect_dump_count][15]  = 0;
				value_dump[ss_collect_dump_count][16]  = 0;
				value_dump[ss_collect_dump_count][17]  = 0;
				value_dump[ss_collect_dump_count][18]  = 0;
				value_dump[ss_collect_dump_count][19]  = 0;
	        	
	    
	        	
	        	if(ss_collect_dump_count == 0){
	        	    ss_collect = 0;
	        	}
	    
	 
	    
	    
	    
	}
	  
	    
	if (ss_collect == 3.0f){
	    
	    ss_collect_dump_count = log_num_samples;
	    
	    ss_collect= 2;
	}    
	    
	    
	    
	    
	#endif    
	    
	    
	    
	    

	
	/**************************************************************************/
	/**************************************************************************/
	/**     Output Vectors on D/A											 **/
	/**	 																	 **/
	/**	 																	 **/
	/**************************************************************************/
	/**************************************************************************/
	#ifdef _DA_OUTPUT

	AN[0] = 0.1f;//v_dqs_s_star_2.q/26.7f;//i_dqs_s_2.q/26.7f;//v_dc_link/267.0f;// //omega_dot_filt_kp1_1*drivecycle_ja_1;//tem_star_1; //v_dqs_s_star_2.q *0.010f;   //lambda_dqs_s_hat_kp1_2.q;//This was q axis stator voltage, now changes to debug speed trajectory  drivecycle_speed_2*0.010f;//
	AN[1] = 0.2f;//v_dqs_s_star_2.d/26.7f;//i_dqs_s_2.d/26.7f;//v_dqs_s_star_foc_2.d/26.0f;// //tem_hat_kp1_2;//-v_dqs_s_star_2.d *0.010f;  //lambda_dqs_s_hat_kp1_2.d;//omega_star_kp1_2*0.010f;//
	AN[2] = 0.2f;//i_dqs_s_2.q/26.7f;//i_dqs_s_1.q/26.7f;
	AN[3] = 0.2f;//i_dqs_s_2.d/26.7f;//i_dqs_s_1.d/26.7f;
	AN[4] = 0.2f;//tem_star_1/26.7f;
	AN[5] = 0.3f;//tem_star_2/26.7f;
	AN[6] = 0.4f;//tem_crank/2.60f; //tem_crank/26.7f; 
	AN[7] = 0.5f;//-lambda_dqr_s_hat_kp1_2.d*3.0f; 

	da_convert(AN0,AN07);
	da_trigger(AN0_15_);
	
	
	#endif  
	
	if (flag_indicator!=drive_flag)
 {
 if (drive_flag == RUN)
 {
	  display_clear();
	  display_putstring("Status RUN. ");
  
	//  if (PWM_D.HOLDENABLE == 0xFF) //PWM_B.HOLDENABLE == 0xFF
	//  {
	//     display_putstring("load inverter off "); 
	//  }
  
	  if (PWM_B.HOLDENABLE == 0xFF)  //PWM_C.HOLDENABLE == 0xFF
	  {
	     display_putstring("test inverter off "); 
	  } 
 }
 
  if (drive_flag == STOP)
 {
	  display_clear();
	  display_putstring("Status STOP. "); 
  
	//  if (PWM_D.HOLDENABLE == 0xFF)  //PWM_B.HOLDENABLE == 0xFF
	//  {
	//     display_putstring("load inverter off "); 
	//  }
  
	  if (PWM_B.HOLDENABLE == 0xFF)   //PWM_C.HOLDENABLE == 0xFF
	  {
	     display_putstring("test inverter off "); 
	  }
	  switch(test1)
	  {
	  	case 1:
	  	{
	  		display_putstring("load motor current failure "); 
	  	}
	  	case 2:
	  	{
	  		display_putstring("test motor current failure "); 
	  	}
	  	/*case 3:
	  	{
	  		display_putstring("DC bus voltage failure "); 
	  	}*/
	  }
   
 }
  
  if (drive_flag == READY)
 {
	  display_clear();
	  display_putstring("Status READY. ");
  
 }
  
  if (drive_flag == STARTUP)
 {
	  display_clear();
	  display_putstring("Status STARTUP. ");
  
 } 
    
 }


 flag_indicator=drive_flag;
  
	    	
}








/**************************************************************************/
/**************************************************************************/
/** name:	  link_isr													 **/
/**																		 **/
/** function: Update the data link                                    	 **/
/**		                                                      			 **/
/**************************************************************************/
/**************************************************************************/
void link_isr(irq_param) 
{
    
    #ifdef _DATALINK
		link.Update();
	#endif
/*	
	#ifdef _AIXSCOPE
	AixScope_update();
	#endif
*/
	/*			REMOVED FOR SPACE - TRO 11/2009

	// Run Menu at much slower rate
	menu_count +=1;
	if( menu_count == menu_decimate){
	   menu_count = 0;
	   menu_serve();
	   
	} 
	*/
	
}


void enc_isr(irq_param)
{
}



/**************************************************************************/
/**************************************************************************/
/** name:	  init														 **/
/**																		 **/
/** function: Init Routine for setting up all components of the system	 **/
/**			  here the user once declares his own configuration			 **/
/**************************************************************************/
/**************************************************************************/
void init(void){

//	delay (0.1f);
	// Initialize Menu for char display
//	menu_init();
/*	
	// init AIX scope 
	#ifdef _AIXSCOPE
	AixScope_init();
	#endif
*/
	// For Data Linker
	#ifdef _DATALINK
		link.Open();
	#endif
	
	
	// disable all LED on the XCP2000
	LED_off(0xFF);
	// init serial port
	serial_init(2);
	// disable Write protection for all secured configuration registers
	write_protect_off();
	// disable all PWM channels
	#ifdef _PWM_C
		pwm_stop(PWM_C);
		// Initialize PWM C
		init_pwm (PWM_C, pwm_frequency, pwm_deadtime);
		set_port (PORT_C,0x00); // out value
		enable_output_port(PORT_C,0xFF); // dir to output
		// Synch PWM to interrupt
		pwm_config_lso(PWM_C, LSYNC0, false, true);
	#endif	
	//pwm_stop(PWM_D);
	#ifdef _PWM_B
		pwm_stop(PWM_B);	
		// Initialize PWM B
		init_pwm (PWM_B, pwm_frequency, pwm_deadtime);
		set_port(PORT_B,0x00); // out value
		enable_output_port(PORT_B,0xFF); // dir to output
		// Synch PWM to interrupt
		pwm_config_lso(PWM_B, LSYNC0, false, true);
	#endif
	
	// Initialize Port A & D	
//	set_port(PORT_A,0x2F);
	enable_output_port(PORT_A,0x00); // dir to input
	//set_port (PORT_D,0x00); // out value
	//enable_output_port(PORT_D,0xFF); // dir to output



	
	// Synch AD conversion to PWM signal generation 
	adda_config_lsiad(AN0_15,  true, true, LSYNC0);
	adda_config_lsiad(AN16_31,  true, true, LSYNC0);
	// Set up Synchronization to IRQ
	AN0_15.ADready.enable();
	AN16_31.ADready.enable();
	// setup Interrupthandler of AD-Converter
	AN0_15.ADready >> IRQ0;
	IRQ0 >> control_ISR;
	AN16_31.ADready >> IRQ0;
	
	
	// service AD-Ready interrupt flag before enabling the system
	AN0_15.ADready.ack();
	AN16_31.ADready.ack();
	// now enable System	
	msc_clear_shutdown();
	// Set up data transfer interrupt
	timer0_stop();
	timer0_init(link_isr,1.0f/link_frequency);
	timer0_start();
	
	//Init incremental Encoder
	ENCOD_A.IRQ1 >> IRQ1;
	IRQ1 >> enc_isr;
	ENCOD_A.IRQ1.ack();
			
	ENCOD_A.CONF_A.SM = true;
	ENCOD_A.CONF_A.OM = true;
	ENCOD_A.CONF_B.SM = true;
	ENCOD_A.CONF_B.OM = true;
	ENCOD_A.LSICFG.SELECT = 1; 
	ENCOD_A.LSICFG.RISING = true;
	ENCOD_A.LSICFG.FALLING = true;
	
		// start PWM block
	pwm_start(PWM_B);
	// start PWM block
	pwm_start(PWM_C);
	
	// enable Write protection for all secured configuration registers
	write_protect_on();
	// Init I2C
	i2c.init();
	
	
	drive_flag = INIT;
	display_clear();
	
}







/**************************************************************************/
/**************************************************************************/
/** name:	  main_loop													 **/
/**																		 **/
/** function: this loop runs at the maximum speed that is left after all **/
/**			  interrupt processes are served. speed may vary depending on**/
/**			  the amount of computational power required in interrupts.	 **/
/**************************************************************************/
/**************************************************************************/
void main_loop(void)
{
   UnionData data;

 //  	char I2C_DataTX;
//	const float I2C_delay = 0.000002f;		

	delay(0.1f);
	int status = link.Read( &command, sizeof( command ) );
	cmd = command.ID;
	data.uint = command.Data;
	cmd_data = data.f;
	if( status != sizeof( command ) )
	 {
	    command.ID=0;
		return;
	}


	
	switch( cmd ) {
	    case CMD_NOP:
	    {
	        LED_on(0x01);
	        break;
	    }
	    case CMD_STOP:
	    {
	        drive_flag = STOP;
	        break;
	        		}
	    case CMD_START:
	    {
	        drive_flag = STARTUP;
	        break;
		}
		case CMD_READY:
	    {
	       drive_flag = READY;
	       break;
		}
	    case CMD_SEND_DATA:
	    {
			link.Write( (const void*)&value_dump, log_num_channels*log_data_length);
			break;
	    }
	   	case CMD_SEND_DATA_SIZE:
	    {
	        command.ID   = CMD_SEND_DATA_SIZE;
			command.Data = log_data_length;
			link.Write( &command, sizeof( command ) );
			break;
	    }	    
	    case CMD_SEND_NUM_CHAN:
	    {
			command.ID   = CMD_SEND_NUM_CHAN;
			command.Data = log_num_channels;
			link.Write( &command, sizeof( command ) );
			break;

	    }
	    case CMD_REC_FLAG:
	    {
			command.ID   = CMD_REC_FLAG;
			flag_rec=(int)cmd_data;
			break;

	    }
	    case CMD_REC_DECIMATE:
	    {
			command.ID   = CMD_REC_DECIMATE;
			log_sample_decimate=(int)cmd_data;
			break;

	    }
	    case CMD_DATASET:
	    {
			command.ID   = CMD_DATASET;
			dataset=cmd_data;
			break;

	    }
	    case CMD_OMEGA_STAR_1:
	    {
			command.ID   = CMD_OMEGA_STAR_1;
			omega_star_kp1_1=cmd_data;
			break;

	    }

	    /*
	    case CMD_BA_1:
	    {
			command.ID   = CMD_BA_1;
			ba_1 =cmd_data;
			break;

	    }
	    
	    case CMD_KSA_1:
	    {
			command.ID   = CMD_KSA_1;
			//ksa_1=cmd_data;
			f_0 = cmd_data;
			break;

	    }
    
	    case CMD_KISA_1:
	    {
			command.ID   = CMD_KISA_1;
			//kisa_1=cmd_data;
			f_1 = cmd_data;
			break;

	    }
	    */
	    case CMD_BA2_1:
	    {
			command.ID   = CMD_BA2_1;
			omega_err_sqared_1=cmd_data;
			break;

	    }
	    
	    case CMD_JA_1:
	    {
			command.ID   = CMD_JA_1;
			ja_1=cmd_data;
			break;

	    }
	    
		case CMD_DRIVECYCLE_JA_1:
	    {
			command.ID   = CMD_DRIVECYCLE_JA_1;
			drivecycle_ja_1=cmd_data;
			break;

	    }
	    
	    case CMD_DRIVECYCLE_DRAG_1:
	    {
			command.ID   = CMD_DRIVECYCLE_DRAG_1;
			drivecycle_drag_1=cmd_data;
			break;

	    }
	    
	    case CMD_DRIVECYCLE_RESISTANCE_1:
	    {
			command.ID   = CMD_DRIVECYCLE_RESISTANCE_1;
			drivecycle_resistance_1=cmd_data;
			break;

	    }
	    
	    case CMD_FOC_FLUX_STAR_1:
	    {
			command.ID   = CMD_FOC_FLUX_STAR_1;
			lamda_star_1 = cmd_data;
			break;

	    }

	    case CMD_OMEGA_SELECT_2:
	    {
			command.ID   = CMD_OMEGA_SELECT_2;
			omega_star_select_2= cmd_data;
			break;

	    }
		/*    
	    case CMD_BA_2:
	    {
			command.ID   = CMD_BA_2;
			ba_2=cmd_data;
			break;

	    }
	    
	    case CMD_KSA_2:
	    {
			command.ID   = CMD_KSA_2;
			ksa_2=cmd_data;
			break;

	    }
	    
	    case CMD_KISA_2:
	    {
			command.ID   = CMD_KISA_2;
			kisa_2=cmd_data;
			break;

	    }
	    */
	    case CMD_OMEGA_REF_2:
	    {
			command.ID   = CMD_OMEGA_REF_2;
			omega_star_const_2=cmd_data;
			break;

	    }
	    
	    case CMD_OMEGA_SINE_OFFSET_2:
	    {
			command.ID   = CMD_OMEGA_SINE_OFFSET_2;
			omega_star_sin_offset_2=cmd_data;
			break;

	    }
	    
	    case CMD_OMEGA_SINE_AMP_2:
	    {
			command.ID   = CMD_OMEGA_SINE_AMP_2;
			omega_star_sin_amp_2=cmd_data;
			break;

	    }
	    case CMD_OMEGA_SINE_FREQ_2:
	    {
			command.ID   = CMD_OMEGA_SINE_FREQ_2;
			omega_star_sin_freq_2=cmd_data;
			break;

	    }
	    
	    case CMD_TORQUE_SINE_OFFSET_1:
	    {
			command.ID   = CMD_TORQUE_SINE_OFFSET_1;
			tem_star_sin_offset_1=cmd_data;
			break;

	    }
	    
	    case CMD_TORQUE_SINE_AMP_1:
	    {
			command.ID   = CMD_TORQUE_SINE_AMP_1;
			tem_star_sin_amp_1=cmd_data;
			break;

	    }
	    case CMD_TORQUE_SINE_FREQ_1:
	    {
			command.ID   = CMD_TORQUE_SINE_FREQ_1;
			tem_star_sin_freq_1=cmd_data;
			tem_star_dyn_nSample = 1/tem_star_sin_freq_1/dt;
			break;

	    }
	    
	    //added by BFB for Flux Sine Traj
	    
	    case CMD_LAMBDA_SINE_OFFSET_2:
	    {
			command.ID   = CMD_LAMBDA_SINE_OFFSET_2;
			lambda_s_star_sin_offset_2=cmd_data;
			break;

	    }
	    
	    case CMD_LAMBDA_SINE_AMP_2:
	    {
			command.ID   = CMD_LAMBDA_SINE_AMP_2;
			lambda_s_star_sin_amp_2=cmd_data;
			break;

	    }
	    case CMD_LAMBDA_SINE_FREQ_2:
	    {
			command.ID   = CMD_LAMBDA_SINE_FREQ_2;
			lambda_s_star_sin_freq_2=cmd_data;
			break;

	    }
	    
	    // end of additions
		
	    
	    case CMD_OMEGA_SLEW_2:
	    {
			command.ID   = CMD_OMEGA_SLEW_2;
			omega_slew_2 =cmd_data;
			break;

	    } 
	    

	    case CMD_TORQUE_SLEW_2:
	    {
			command.ID   = CMD_TORQUE_SLEW_2;
			tem_star_slew_2 =cmd_data;
			break;

	    } 
	  	    
	    case CMD_TORQUE_MOD_SELECT_2:
	    {
			command.ID   = CMD_TORQUE_MOD_SELECT_2;
			torq_mod_select=cmd_data;
			break;

	    }
	    
	    case CMD_TORQUE_MOD_SELECT_1:
	    {
			command.ID   = CMD_TORQUE_MOD_SELECT_1;
			torq_mod_select_1=cmd_data;
			break;

	    }
	    
	    case CMD_LOAD_TRAJ_SELECT_1:
	    {
			command.ID   = CMD_LOAD_TRAJ_SELECT_1;
			dyn_load_select_num=cmd_data;
			break;

	    }
	    
	    case CMD_CYCLICAL_FREQ:
	    {
			command.ID   = CMD_CYCLICAL_FREQ;
			Cyclical_freq =cmd_data;
			tem_star_dyn_nSample = 1/Cyclical_freq/dt;
			break;

	    }
	    
	    case CMD_DYN_LOAD_NCYCLES:
	    {
			command.ID   = CMD_DYN_LOAD_NCYCLES;
			dyn_load_nCycles =cmd_data;
			break;

	    }
    
	    case CMD_VS_AMP_2:
	    {
			command.ID   = CMD_VS_AMP_2;
			v_exc_amp_2=cmd_data;
			break;

	    }
	    
	    case CMD_VS_FREQ_2:
	    {
			command.ID   = CMD_VS_FREQ_2;
			v_exc_freq_2=cmd_data;
			break;

	    }
	    
	    case CMD_VS_AMP_1:
	    {
			command.ID   = CMD_VS_AMP_1;
			v_exc_amp_1=cmd_data;
			break;

	    }
	    
	    case CMD_VS_FREQ_1:
	    {
			command.ID   = CMD_VS_FREQ_1;
			v_exc_freq_1=cmd_data;
			break;

	    }
		    
	    case CMD_FOC_REG_P_2:
	    {
			command.ID   = CMD_FOC_REG_P_2;
			k_p_ireg_2=cmd_data;
			break;

	    }
	    
	    case CMD_FOC_REG_I_2:
	    {
			command.ID   = CMD_FOC_REG_I_2;
			k_p_ireg_2=cmd_data;
			break;

	    }
    
	    case CMD_FOC_FLUX_STAR_2:
	    {
			command.ID   = CMD_FOC_FLUX_STAR_2;
			lamda_star_2=cmd_data;
			break;

	    }
		    
	    case CMD_DB_FLUX_SELECT_2:
	    {
			command.ID   = CMD_DB_FLUX_SELECT_2;
			lambda_s_star_select_2=cmd_data;
			break;

	    }
	    
	    case CMD_DB_FLUX_STAR_2:
	    {
			command.ID   = CMD_DB_FLUX_STAR_2;
			lambda_s_star_cmd_2=cmd_data;
			lambda_s_star_cmd_pre_2 = cmd_data;		//Variable to maintain original value of commanded flux linkage
			break;

	    }

	    case CMD_DB_FLUX_TAU_2:
	    {
			command.ID   = CMD_DB_FLUX_TAU_2;
			lambda_db_tau=cmd_data;
			break;

	    }

	    
	    case CMD_DB_FLUX_SLEW_2:
	    {
			command.ID   = CMD_DB_FLUX_SLEW_2;
			lambda_db_slew=cmd_data;
			break;

	    }
	    
	    // inverter compensation improvement
	    /*
	    case CMD_INV_COMP_REF_T_2:
	    {
			command.ID   = CMD_INV_COMP_REF_T_2;
			pwm_t_comp=cmd_data;
			break;

	    }


	    
	    
	    case CMD_INV_COMP_2:
	    {
			command.ID   = CMD_INV_COMP_2;
			pwm_inv_comp=cmd_data;
			break;

	    }

	    
	    case CMD_INV_V_FWD_DROP_2:
	    {
			command.ID   = CMD_INV_V_FWD_DROP_2;
			v_fwd_drop=cmd_data;
			break;

	    }
	    */
	    
	    /*
	    case CMD_SS_COLLECT_FLAG:
	    {
			command.ID   = CMD_SS_COLLECT_FLAG;
			ss_collect=cmd_data;
			break;

	    }
	    */
	    
	     case CMD_DYN_MACRO_CMD:
	    {
			command.ID   = CMD_DYN_MACRO_CMD;
			dyn_macro_cmd=cmd_data;
			break;

	    }
	    
	     case CMD_DYN_MACRO_LAMBDA_STAR:
	    {
			command.ID   = CMD_DYN_MACRO_LAMBDA_STAR;
			dyn_macro_lambda_star=cmd_data;
			break;

	    }
	    
	     case CMD_DYN_MARCO_PRETRIG:
	    {
			command.ID   = CMD_DYN_MARCO_PRETRIG;
			dyn_macro_pretrig=(int)cmd_data;
			break;

	    }
	     case CMD_DYN_MARCO_OMEGA_STAR:
	    {
			command.ID   = CMD_DYN_MARCO_OMEGA_STAR;
			dyn_macro_omega_star=cmd_data;
			break;

	    }
	    
	    case CMD_DYN_MARCO_TORQ_CMD:
	    {
			command.ID   = CMD_DYN_MARCO_TORQ_CMD;
			dyn_macro_torque_cmd=cmd_data;
			break;

	    }
	    
	    case CMD_DYN_MARCO_TORQ_SLEW:
	    {
			command.ID   = CMD_DYN_MARCO_TORQ_SLEW;
			dyn_macro_torque_slew=cmd_data;
			break;

	    }
	    
	    case CMD_DYN_MARCO_TORQ_OFFSET:
	    {
			command.ID   = CMD_DYN_MARCO_TORQ_OFFSET;
			dyn_macro_torque_offset=cmd_data;
			break;

	    }
	    case CMD_DYN_MARCO_TORQ_CMD_NXT:
	    {
			command.ID   = CMD_DYN_MARCO_TORQ_CMD_NXT;
			dyn_macro_torque_cmd_nxt=cmd_data;
			break;

	    }
	    
	    case CMD_FOC_TORQUE_STAR_1:
	    {
			command.ID   = CMD_FOC_TORQUE_STAR_1;
			tem_star_1 = cmd_data;
			break;

	    }
	    
	    case CMD_FOC_TORQUE_STAR_2:
	    {
			command.ID   = CMD_FOC_TORQUE_STAR_2;
			tem_star_2 = cmd_data;
			break;

	    }
	    
	    case CMD_OMEGA_STAR_2:
	    {
			command.ID   = CMD_OMEGA_STAR_2;
			omega_star_kp1_2=cmd_data;
			break;

	    }

	    case CMD_FOC_IQS_E_STAR_1:
	    {
			command.ID   = CMD_FOC_IQS_E_STAR_1;
			i_dqs_e_star_1.q = cmd_data;
			break;

	    }
	    
	    case CMD_FOC_IDS_E_STAR_1:
	    {
			command.ID   = CMD_FOC_IDS_E_STAR_1;
			i_dqs_e_star_1.d = cmd_data;
			break;

	    }
	    
	    case CMD_FOC_IQS_E_STAR_2:
	    {
			command.ID   = CMD_FOC_IQS_E_STAR_2;
			#ifdef _CHIRP
				flag_start_chirp = cmd_data;//i_dqs_e_star_2.q = cmd_data;
			#endif
			i_dqs_e_star_2.q = cmd_data;
			break;

	    }

	    #ifdef _CHIRP
	    case CMD_CHIRP_DIRECTION:
	    {
	    	command.ID = CMD_CHIRP_DIRECTION;
	    	posneg = cmd_data;
	    }
	    #endif

		#ifdef _NOISE
	    case CMD_NOISE:
	    {
	    	command.ID = CMD_NOISE;
	    	flag_start_noise = cmd_data;
	    	break;
	    }
	    #endif
	    
	    case CMD_FOC_IDS_E_STAR_2:
	    {
			command.ID   = CMD_FOC_IDS_E_STAR_2;
			i_dqs_e_star_2.d = cmd_data;
			break;

	    }
	    
	    #ifdef _SQUARE_TORQUE
		    case CMD_DB_TORQUE_STAR_SQURE_2:
		    {
		    	command.ID = CMD_DB_TORQUE_STAR_SQURE_2;
		    	square_torque_flag = cmd_data;
	    
		    }
		#endif
		#ifdef _SINE_WAVE
			case CMD_FOC_IQS_SINE_WAVE_2:
			{
				command.ID = CMD_FOC_IQS_SINE_WAVE_2;
				sine_wave_flag = cmd_data;
			}
			/*case CMD_FOC_IQS_SINE_WAVE_FREQ_2:
			{
				command.ID = CMD_FOC_IQS_SINE_WAVE_FREQ_2;
				sine_freq = (float)cmd_data;
			}
			case CMD_FOC_IQS_SINE_WAVE_AMP_2:
			{
				command.ID = CMD_FOC_IQS_SINE_WAVE_AMP_2;
				sine_amp = (float)cmd_data;
			}*/
			#endif
			case CMD_CURRENT_OBS_LOW_2:
			{
				command.ID = CMD_CURRENT_OBS_LOW_2;
				current_obs_low = cmd_data;
			}
		
/*
		case CMD_PWM_FREQUENCY_2:
		{
			command.ID = CMD_PWM_FREQUENCY_2;
			PWM_frequency_divisor = cmd_data;
			dt = PWM_frequency_divisor/pwm_frequency;
			#ifdef _CHIRP
				f_1 = 0.5/dt;
				Df_by_T_c = (f_1 - f_0)/T_c/2; 
			#endif
			//t_obsr_1 = delta_theta_mult*dt;
			t_obsr_2 = delta_theta_mult*dt;
			//t_sfb_1	= delta_theta_mult*dt;	
			t_sfb_2	= delta_theta_mult*dt;
			//delta_theta_max = omega_max*delta_theta_mult*dt;
		}*/
	    default :
	    {

	        break;
	    }

	}

}







/**************************************************************************/
/**************************************************************************/
/** name:	  rotate													 **/
/**																		 **/
/** function: rotate a dq vector quantity input by angle				 **/
/**			  direction of rotation is determined by sign of angle		 **/
/**			  a positive angle rotates forward (eg the carrier voltage	 **/
/**			  in the stationary frame rotated by theta_c would be DC)	 **/
/**************************************************************************/
/**************************************************************************/
static inline struct dq rotate(struct dq input, float angle)
{
    struct dq output;
    float c,s;
    
    c = cosf(angle);
    s = sinf(angle);
    output.d = input.d*c - input.q*s;
    output.q = input.d*s + input.q*c;
    return output;
    
}


static inline float phase_in_two_pi(float Vqs, float Vds)
{
	float phase;
	if ((Vqs==0.0f)&&(Vds==0.0f))
	phase=0.0f;
	else if ((Vqs==0.0f)&&(Vds>0))
	phase=3*pi_over_2;
	else if ((Vqs==0.0f)&&(Vds<0))
	phase=pi_over_2;
	else 
	{
	phase=atan(-Vds/Vqs);
	if (Vqs<0)
	phase=phase+pi;
	else if(Vds>0)
	phase=phase+2*pi;
	}
	/*
	if((Vqs==0.0f)&&(Vds>0.0f))
	phase=3*pi/2;
	
	if((Vqs==0.0f)&&(Vds<0.0f))
	phase=1*pi/2;
	*/
	
	return phase;
}







