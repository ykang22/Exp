#include"hardware.h"

// Global Variables
unsigned         timerVal;
float            position, velocity, err, vel_star, d_unsat, d_star, vel_manip, va_star;
float            Vdc = 7.1, km, kchop, k;
float            kp = 1, ki = 5, kd = 0, k0, k1, k2;
float            position_past, d_past, err_past, vel_manip_past;
float            pwm_frequency, pwm_period, timer_frequency, timer_period, pwm_deadtime;
float            Vsource, Vdrain, Va, Vshunt, Vds, Ids, Rds, Rds_nom, Tj_hat;

// Define test variables to determine analog input pin mapping
// int signal_1,signal_2,signal_3,signal_4,signal_5,signal_6,signal_7;
// int signal_8,signal_9,signal_10,signal_11,signal_12,signal_13,signal_14;

void timer_interrupt(irq_param param){

	// Analog to Digital Conversion
	AN0_15.ADready.ack();     // Resets the interrupt flag of the ADC unit for channels 0-15
	ad_convert(AN08,AN15);     // Calibrates the analog inputs corresponding to channels 0-15
	AN16_31.ADready.ack();    // Resets the interrupt flag of the ADC unit for channels 16-31	
	ad_convert(AN16,AN31);    // Calibrates the analog inputs corresponding to channels 16-31
	
	// Assign AD values to a variable
	/*Vsource  = AN[AN08];
	Va       = AN[AN09];
	Vdrain   = AN[AN12];*/
	
	
	timerVal++;
	//display_clear();
	
	// Create a sinusoidal velocity command
	k = timerVal;
	vel_star = 1 + 0.5*sin(k*pwm_period);
	// vel_star = 1;
	// vel_star = 0;
/*	
	// Calculate voltage drops across MOSFET and shunt resistor
	Vds = Vsource - Vdrain;
	Vshunt = Vdrain - Va;
	
	// Calculate current and Rds using the current
	Ids = Vshunt*10;                                   // Idson = Vshunt/Rshunt where Rshunt = 0.1 ohm
	Rds = Vds/Ids;                                     // Calculate the resistance
	// Tj_hat = 0.7 + 0.196668*exp(0.01289*Rds);       // Estimate the junction temperature using a curve fit of data sheet information
	Tj_hat = 71.328*log(Rds - 0.7) + 120.1855;    // Estimate the junction temperature using a curve fit of data sheet information
	*/
	// Measure and display DC machine position using encoder feedback
	ENCOD_A.TRG = true;
	position = ENCOD_A.POS_A;
	position = (position/1920); // revs
	//display_putint_dec(position);
	
	// Calculate average velocity of the DC machine using encoder feedback
	velocity = (position - position_past)*pwm_frequency; //Hz
	
	// Calculate the error between velocity command and average velocity
	err = vel_star - velocity;
	
	// Execute control law calculations to determine required manipulation
	vel_manip = vel_manip_past + k0*err + k1*err_past;
	va_star = vel_manip/km;
	d_unsat = va_star/kchop;	
	
	// Saturate the manipulated input duty ratio
	if( d_unsat >= 1){d_star = 1;}
	else if( d_unsat <= 0){d_star = 0;}
	else d_star = d_unsat;
	
	// Define history variables: position and error
	position_past = position;
	err_past = err;                        // Define a past error value for the control law
	vel_manip_past = vel_manip;            // Define a past manipulation value for the control law
	
	// Allow PWM to operate again and set duty cycle of all pairs to 50% 
	pwm_set_combined_dutycycle(PWM_B.PAIR0, 0.5);
	pwm_set_combined_dutycycle(PWM_C.PAIR0, 0.5);
	
	// Analog to Digital Conversion
	AN0_15.ADready.ack();     // Resets the interrupt flag of the ADC unit for channels 0-15
	ad_convert(AN08,AN15);     // Calibrates the analog inputs corresponding to channels 0-15
	AN16_31.ADready.ack();    // Resets the interrupt flag of the ADC unit for channels 16-31	
	ad_convert(AN16,AN31);    // Calibrates the analog inputs corresponding to channels 16-31
	
	// Assign AD values to a variable
	Vsource  = AN[AN19];
	Va       = AN[AN23];
	Vdrain   = AN[AN12];
	
		// Create a sinusoidal velocity command
	k = timerVal;
	vel_star = 1 + 0.5*sin(k*pwm_period);
	// vel_star = 1;
	// vel_star = 0;
	
	// Calculate voltage drops across MOSFET and shunt resistor
	Vds = Vsource - Vdrain;
	Vshunt = Vdrain - Va;
	
	// Calculate current and Rds using the current
	Ids = Vshunt*10;                                   // Idson = Vshunt/Rshunt where Rshunt = 0.1 ohm
	Rds = Vds/Ids;                                     // Calculate the resistance
	// Tj_hat = 0.7 + 0.196668*exp(0.01289*Rds);       // Estimate the junction temperature using a curve fit of data sheet information
	Tj_hat = 71.328*log(Rds - 0.7) + 120.1855;    // Estimate the junction temperature using a curve fit of data sheet information
	
		
	// Display feedback information
	if(KEY.pressed(KEY_F1) == true)
	{
		display_clear();
		DISPLAY.gotoxy(0,0); display_putint_dec(Vsource);
		DISPLAY.gotoxy(0,1); display_putint_dec(Va);
		DISPLAY.gotoxy(0,2); display_putint_dec(Vdrain);
		

		/*
		DISPLAY.gotoxy(0,0); display_putint_dec(1000*Vsource);
		DISPLAY.gotoxy(0,1); display_putint_dec(1000*Vdrain);
		DISPLAY.gotoxy(0,2); display_putint_dec(1000*Va);
		DISPLAY.gotoxy(0,3); display_putint_dec(Tj_hat);
		*/
	}
};

void init(void){
	/*LED_off(0xFF);
	display_putstring("Greetings, Human!");*/
	
	// Define timing constants
	pwm_frequency = 100;                 // [Hz]
	pwm_period = 1/pwm_frequency;        // [s]
	timer_frequency = pwm_frequency;     // [Hz]
	timer_period = 1/timer_frequency;    // [s]
	// timer_period = 0.01f;
	// pwm_frequency = 1/timer_period;
	pwm_deadtime = 1e-6;
	//PWM_B.FREQ = pwm_frequency;
	
	timerVal = 0;
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
	
	// Initialize Port A	
	enable_output_port(PORT_A,0x00); // dir to input
	
	// Synch PWM to interrupt
	pwm_config_lso(PWM_B, LSYNC0, true, true);
	//pwm_config_lso(PWM_C, LSYNC0, true, true);
	
	// Synch AD conversion to PWM signal generation 
	adda_config_lsiad(AN0_15,  true, false, LSYNC0); //true true (assymmetrical PWM) or true false (symmertical PWM)
	adda_config_lsiad(AN16_31, true, false, LSYNC0); 

	// Set up Synchronization to IRQ
	AN0_15.ADready.enable();
	AN16_31.ADready.enable();

	// setup Interrupthandler of AD-Converter
	IRQ0 >> timer_interrupt;
	AN0_15.ADready >> IRQ0;
	AN16_31.ADready >> IRQ0;

	// service AD-Ready interrupt flag before enabling the system
	AN0_15.ADready.ack();
	AN16_31.ADready.ack();
	
	delay(1);
	
		// Start PWM	
	pwm_start(PWM_B);
	// start PWM block
	pwm_start(PWM_C);
	
	//assert(XCI(Slot7).write(Slot7));
	// now enable System	
	msc_clear_shutdown(); // Clears shutdown bit
	
	
	write_protect_on();
	display_clear();
	
	// Define physical system gains
	kchop = Vdc;                        // This is the gain of the chopper circuit [V/-]
	km = 0.4861;                        // This is the DC machine gain from its data sheet [Hz/V]
	
	// Define controller gains
	k0 = kp + ki*pwm_period + kd*pwm_frequency;
	k1 = -kp - 2*kd*pwm_frequency;
	k2 = kd*pwm_frequency;
	
	// Define testing parameters
	k = timerVal;
	vel_star = 0;             // [Hz]
	Rds_nom = 8;              // [Ohm]

	
	// Begin the Timer 0 interrupt
	timer0.start();	
};

void main_loop(void){
	
	
};
