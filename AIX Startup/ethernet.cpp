/*****************************************************************************/
/*                Ethernet Datalink Program - ethernet.cpp					 */
/*																			 */
/*    Description: This file contains the implementation of the 			 */
/*				   ethernet interface for the XCS2000						 */
/*																			 */
/* 	  v1.0		Based on Bjorn Heinbokel and Mike Leetmaa's code			 */
/*																			 */
/*                     Matt Burton 2008 (WEMPEC)							 */
/*****************************************************************************/

#include "ethernet.h"
//#include "constants.c"
#include "XCSDataLink.h"
#include <terminal.h>
#include "hardware.h"


// instance of the class XCSDataLink used to access XCSDataLink.dlb  
DataLink::XCSDataLink link;

//Flags used in main
//volatile int  flagRec = 0;
//volatile long dump_cnt = 0;


//DataLink variables used in main
Command command;
UnionData data;


//Data logging variable in SRAM
_Pragma( "section( \"/NO_INIT seg_SDRAM_NOINIT\" )" )
_Pragma( "section( \"seg_SRAM\" )" )
float Value_dump[num_samples][num_channels];

//------- Functions ----------\\

void Initialize()
{
    link.Open();
}

void Wait()
{
    delay (0.1f);
    term << "Waiting for command..." << endl;
    delay(0.2f);
	const int status = link.Read( &command, sizeof(command) );
	if( status != sizeof(command) ) {
		term << "Error in Read (";
		term << status << ")!" << endl;
		return;
	}
	term << "OK." << endl;
}

void NOP()
{
    term << "CMD_NOP" << endl;
}
/*
void StartExperiment()
{
    LED_off(0x0F);
    wait = false;
	startup = true;
	//experiment = true;
//	if(fault){
//		flagRec = 1.f;
//	}
	term << "CMD_START_EXP" << endl;
}

void SetLogData()
{
    data.uint = command.Data;
    flagLog = data.f;
	term << "CMD_SET_LOGDATA" << endl;
}

void ExpDone()
{
    command.ID   = CMD_EXP_DONE;
	command.Data = 0;
	LED_on(0x01);
	term << "Writing experiment done" << endl;
	link.Write( &command, sizeof( command ) );
}

void SpeedProfile()
{
    term << "CMD_SPD_PROF" << endl;
	data.uint = command.Data;
	 
	term << "Speed Set to." << endl;
}

void SetVelSS()
{
    term << "CMD_SET_SPD" << endl;
	data.uint = command.Data;
	w_r_star_ss = data.f/30;
	term << "Speed Set to." << w_r_star_ss << endl;
}

void SetLoadFlag()
{
    term << "CMD_SET_LOAD_FLAG" << endl;
	data.uint = command.Data;
	flagLOAD =  data.f;
	term << "Load flag Set"  << endl;
}

void SetLoadTe()
{
    term << "CMD_SET_LOAD_TE" << endl;
	data.uint = command.Data;
	//scale torque command from Nm to analogue voltage out
	T_load_ss = (data.f - 0.08)/0.4885;
	//limit the command to be between 0 and 10 volts
	if(T_load_ss < 0 ) { T_load_ss = 0; }
	if(T_load_ss > 10 ) { T_load_ss = 10; }
	term << "Load Torque Set to." << T_load_ss << endl;
}

void SetTorque()
{
    term << "CMD_SET_TORQUE" << endl;
	data.uint = command.Data;
	T_e_star_ss = data.f;
	term << "Torque Set to ." << T_e_star_ss << endl;
}

void SetFlux()
{
    term << "CMD_SET_FLUX" << endl;
	data.uint = command.Data;
	lambda_s_star_ss = data.f;
	term << "Flux Magnitude Set to ." << lambda_s_star_ss << endl;
}

void ExperimentSS()
{
	term << "CMD_EXP_SS" << endl;
	data.uint = command.Data;
    flagSS = data.f;
    if(flagSS>0){
		flagRec = 0;
    }
	term << "Flagged Steady State" << endl;
}

void ExperimentEfficiency()
{
	term << "CMD_EXP_EFF" << endl;
	data.uint = command.Data;
    flagEFF = data.f;
	term << "Flagged Efficiency Experiment" << endl;
}

void SendData()
{    
    LED_on(0x01);
	term << "Writing data...";
	delay(0.01f);
	link.Write( (const void*)&Value_dump, num_channels*data_length);

}

void SendDataSize()
{    
    command.ID   = CMD_SENDDATASIZE;
	command.Data = data_length;
	LED_on(0x01);
	term << "Writing size: " << command.Data << endl;
	link.Write( &command, sizeof( command ) );
}

void SendNumChan()
{    
    command.ID   = CMD_SENDNUMCHAN;
	command.Data = num_channels;
	LED_on(0x01);
	term << "Writing size: " << command.Data << endl;
	link.Write( &command, sizeof( command ) );
}

void LoadDriveON()
{
    term << "CMD_LOAD_DRIVE_ON" << endl;
	load_drive_on =  true;
	term << "Load Drive ON"  << endl;
}

*/
