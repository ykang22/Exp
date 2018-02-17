/*****************************************************************************/
/*                Ethernet Datalink Program - ethernet.h					 */
/*																			 */
/*    Description: This file contains the interface for ethernet.cpp		 */
/*				   (variables, objects and functions that exist)			 */
/* 				   (				in ethernet.cpp			   )			 */
/*																			 */
/* 	  v1.0		Based on Bjorn Heinbokel and Mike Leetmaa's code			 */
/*																			 */
/*                     Matt Burton 2008 (WEMPEC)							 */
/*****************************************************************************/

#ifndef ETHERNET_H
#define ETHERNET_H

#include "XCSDataLink.h"

//------Datalink Protocol variables------// 

	struct Command { 
	    unsigned ID; 
	    unsigned Data; 
	};
	

	union UnionData { 
	    unsigned uint; 
	    float f; 
	};

	// DataLink Commands are defined here:
	enum { 
		CMD_NOP 	  	  = 1,
		//start stuff 10-19
		CMD_START_EXP 	  = 10,
		CMD_SET_LOGDATA	  = 11,
		CMD_LOAD_DRIVE_ON = 15,
		//set params 20-29
		CMD_SET_SPD		  = 21,
		CMD_SET_LOAD_FLG  = 22,
		CMD_SET_LOAD_TE	  = 23,
//		CMD_SET_TE		  = 22,
//		CMD_SET_FLX		  = 23,
		//user defined flags and funntions 31-39
		CMD_EXP_SS	      = 31,
		CMD_EXP_EFF	      = 32,
		CMD_EXP_DONE      = 35,
		//data trans
		CMD_SEND_DATA     = 101,
		CMD_SENDDATASIZE  = 102,
		CMD_SENDNUMCHAN   = 103	
	};
	
	// Constants defining the data logging space
	const int num_channels      = 10;
	const int num_samples       = 20000;
	extern volatile int data_length;
	
	// DataLink Variables
	extern DataLink::XCSDataLink link;
//	extern bool          log_active;
	extern volatile int  flagRec;
//	extern volatile long dump_cnt;
	extern volatile int  flagSS;
	extern volatile int  flagEFF;
	extern volatile int  flagLOAD;
	extern volatile float flagLog;
	extern volatile bool  load_drive_on;
	extern Command command;
	extern UnionData data;
	extern float Value_dump[num_samples][num_channels];
//	extern volatile float T_e_star_ss;
	extern volatile float T_load_ss;
	extern volatile float w_r_star_ss;
//	extern volatile float lambda_s_star_ss;
	extern bool wait;
	extern bool startup;
//	extern bool experiment;
	extern bool fault;
	
	/*Function Prototypes*/
	void Initialize();
	void Wait();
	void NOP();
	void StartExperiment();
	void SetLogData();
	void SetVelSS();
	void SetLoadFlag();
	void SetLoadTe();
//	void SetTorque();
//	void SetFlux();
	void ExperimentSS();
	void ExperimentEfficiency();
	void ExpDone();
	void SendData();
	void SendDataSize();
	void SendNumChan();
	void LoadDriveON();

#endif /* ETHERNET_H */
