/*************************************************************************/
/*						DBDTFC Control Program - menu.h					 */
/*						v1.0											 */
/*						6/15/2009										 */
/*						Tim Obermann									 */
/*************************************************************************/
// v1.0 Original Development

// Requried Libraries
#include "hardware.h"
//#include <math.h>
//#include <I2C.h>
//#include<xci/xci_send_data.h>
//#include<xci/xci_get_status.h>
//#include<shared/extract.h>


// Variables Displayed on Front Panel
extern volatile float  	v_dc_link;
extern volatile float  	lamda_star_2;
extern volatile float  	tem_star_2;
extern volatile float  	torq_mod_select;
extern volatile float  	torq_mod_select_1;
extern volatile float  	omega_l_hat_kp1_1;
extern volatile float  	lambda_s_star_2;
extern volatile int 	drive_flag;

// Drive States
enum{
    READY,
    STARTUP,
    RUN,
    STOP
};

// Constants
const int shftinc				= 10;	// Multiplier for encoder if shift is pressed.
// Vars
volatile int status_counter 	= 0;	// Keep Alive Status Marker

// function prototypes
void menu_clear_position (void);
void put_float(const float, const unsigned, const unsigned, const bool);




