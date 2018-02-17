#include "main.h"

unsigned timerVal;
float position, velocity, err, vel_star, d_unsat, d_star, vel_manip, va_star;
float Vdc = 7.1, km, kchop, k;
float kp = 1, ki = 5, kd = 0, k0, k1, k2;
float position_past, d_past, err_past, vel_manip_past;
float pwm_period, timer_frequency, timer_period;

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

void timer_interrupt(irq_param param){
	AN0_15.ADready.ack();	
	ad_convert(AN8,AN15);
	AN16_31.ADready.ack();	
	ad_convert(AN16,AN31);
	
	display_clear();
	// Line Current Calculation
	i_a_2 		= AN[AN08]   *2.0f/15.0f 	    + i_a_offset_1; 			// Scaling from Semikron Datasheet
	i_b_2 		= AN[AN12]   *2.0f/15.0f 	    + i_b_offset_1;				// Scaling from Semikron Datasheet
	i_c_2 		= -(i_a_1 + i_b_1);                         				// Scaling from Semikron Datasheet
	i_a_1 		= AN[AN16]   *2.0f/15.0f  		+ i_a_offset_2;				// Scaling from LEM Datasheet (1.75)
	i_b_1 		= AN[AN20]   *2.0f/15.0f  		+ i_b_offset_2;				// Scaling from LEM Datasheet
	i_c_1 		= -(i_a_2 + i_b_2);				                            // Scaling from LEM Datasheet
	
	
	// Calculate DC Link Voltage
    v_dc_link = AN[AN14]*13.0f/2.0f;						    //208.5f // Scaling from Semikron Datasheet

		// Phase Voltage Calculation 
	// Note: v_a_1, v_b_1, v_c_1, etc, are used elsewhere
	v_ab_2_ad 	= AN[AN10] * 32.0f/5.0f; 			        // Scaling
	v_ca_2_ad 	= AN[AN11] * 32.0f/5.0f;			   		// Scaling
	v_bc_2_ad 	= AN[AN12] * 32.0f/5.0f;					// Scaling	
	
	display_putint_dec(v_dc_link);
	DISPLAY.gotoxy(0,1); display_putint_dec(AN[AN10]);
	
}

void init(void){
	LED_off(0xFF);
	
	// Define timing constants
	//pwm_frequency = 100.0f;                 // [Hz]
	pwm_period = 1/pwm_frequency;        // [s]
	timer_frequency = pwm_frequency;     // [Hz]
	timer_period = 1/timer_frequency;    // [s]
	// timer_period = 0.01f;
	// pwm_frequency = 1/timer_period;
//	pwm_deadtime = 1e-6;
	//PWM_B.FREQ = pwm_frequency;
	
	timerVal = 0;
	
	display_putstring("Greetings, Humean!");
	
	timer0.init(timer_interrupt, timer_period);

	#ifdef _DATALINK
	link.Open();
	#endif
	#ifdef _AIXSCOPE
	AixScope_init();
	#endif
	//#define VDSP
	// init serial port
	serial_init(2);
	// disable Write protection for all secured configuration registers
	write_protect_off();
	// disable all PWM channels
	pwm_stop(PWM_B);
	pwm_stop(PWM_C);		
	// Initialize PWM B
	init_pwm(PWM_B, pwm_frequency, pwm_deadtime);
	
	set_port (PORT_B,0x00); // out value
	enable_output_port(PORT_B,0xFF); // dir to output
	// Initialize PWM C
	init_pwm (PWM_C, pwm_frequency, pwm_deadtime);
	set_port (PORT_C,0x00); // out value
	enable_output_port(PORT_C,0xFF); // dir to output
	// Initialize Port A & D	
	enable_output_port(PORT_A,0x00); // dir to input
	// Synch PWM to interrupt
	pwm_config_lso(PWM_B, LSYNC0, true, true);
	//enable_output_port(PORT_A,0x00);
	
	//assert(XCI(Slot7).write(Slot7));
	// now enable System
	// Synch AD conversion to PWM signal generation 
	adda_config_lsiad(AN0_15,  true, false, LSYNC0);
	adda_config_lsiad(AN16_31,  true, false, LSYNC0);
	// Set up Synchronization to IRQ
	AN0_15.ADready.enable();
	AN16_31.ADready.enable();
	// setup Interrupthandler of AD-Converter
	AN0_15.ADready >> IRQ0;
	IRQ0 >> timer_interrupt;
	AN16_31.ADready >> IRQ0;
	// service AD-Ready interrupt flag before enabling the system
	AN0_15.ADready.ack();
	AN16_31.ADready.ack();
	// now enable System	
		
	msc_clear_shutdown(); // Clears shutdown bit
	
	ENCOD_A.CONF_A.OM = true;
	ENCOD_A.CONF_B.OM = true;
	// Start PWM	
	pwm_start(PWM_B);
	// start PWM block
	pwm_start(PWM_C);
	
	
	write_protect_on();
	
	//MSC.SHUTDOWN = false; // Clearing the shutdown bit
	delay(1);
	
	
	// Begin the Timer 0 interrupt	
	timer0.start();	
};

void main_loop(void){
	
};
