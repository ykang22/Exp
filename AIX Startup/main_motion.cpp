#include"hardware.h"

// Global Variables
unsigned timerVal;
float position, velocity, err, vel_star, d_unsat, d_star, vel_manip, va_star;
float Vdc = 7.1, km, kchop, k;
float kp = 1, ki = 5, kd = 0, k0, k1, k2;
float position_past, d_past, err_past, vel_manip_past;
float pwm_frequency, pwm_period, timer_frequency, timer_period, pwm_deadtime;

void timer_interrupt(irq_param param){
	timerVal++;
	
	k = timerVal;
	vel_star = 0;
	//vel_star = 1 + 0.5*sin(k*pwm_period);;
	
	display_clear();
	//DISPLAY.gotoxy(0,1);                 // Why are we doing this?
	
	// Measure and display DC machine position using encoder feedback
	ENCOD_A.TRG = true;
	position = ENCOD_A.POS_A;
	//position = (position/1920); // revs
    DISPLAY.gotoxy(0,0);
 	display_putint_dec(position);
	
	position = ENCOD_A.POS_B;
	//position = (position/1920); // revs
	DISPLAY.gotoxy(0,1);
	display_putint_dec(position);
	
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
	//PWM_B.FORCEENABLE = 0x00;
	//PWM_C.FORCEENABLE = 0x00; 
	pwm_set_combined_dutycycle(PWM_B.PAIR0, 0.5);
	pwm_set_combined_dutycycle(PWM_C.PAIR0, 0.5);
	//DISPLAY.gotoxy(0,2);
	//display_putint_dec(PORT_B.OUT);
	
/*	// Display information for debugging
	DISPLAY.gotoxy(0,0);
	display_putint_dec(100*d_unsat);        // Expressed as a percentage due to factor of 100
	DISPLAY.gotoxy(0,1);
	display_putint_dec(100*d_star);         // Expressed as a percentage due to factor of 100
	DISPLAY.gotoxy(0,2);
	display_putint_dec(10*vel_star);        // Multiply by factor of 10 for viewability
	DISPLAY.gotoxy(0,3);
	display_putint_dec(10*velocity);  */      // Multiply by factor of 10 for viewability
	
		
};

void init(void){
	LED_off(0xFF);
	
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
	
	//PORT_B.OUT[1] = 1;
	//PORT_B.OUT[2] = 1;
	
	// Define physical system gains
	kchop = Vdc;                        // This is the gain of the chopper circuit [V/-]
	km = 0.4861;                        // This is the DC machine gain from its data sheet [Hz/V]
	
	// Define controller gains
	k0 = kp + ki*pwm_period + kd*pwm_frequency;      // 3.451385438
	k1 = -kp - 2*kd*pwm_frequency;                   // -3.451385438
	k2 = kd*pwm_frequency;                           // 0
	
	// Define testing parameters
	k = timerVal;
	vel_star = 1;
	
	// Begin the Timer 0 interrupt	
	timer0.start();	
};

void main_loop(void){
	
};
