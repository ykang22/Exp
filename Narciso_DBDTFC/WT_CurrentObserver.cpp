// MotionObserver.cpp
// Marc Petit
// 04/09/15  

#include "WT_CurrentObserver.h"

/*
	Input:
*/
CurrentObserver::CurrentObserver()
{


	Kco = ComplexVar(C_R_O_QR,C_R_O_QR);
	Kico = ComplexVar(C_SAMPLE_TIME*C_R_IO_QR,C_SAMPLE_TIME*C_R_IO_QR);
//	Kro  = ComplexVar(C_R_RO_QR, C_R_RO_QR);//Kico.scal_mul(C_2sinwcTs_o_wc);
	
	// float tau_d = 0.0004f/0.72f; //C_LS_A/C_RS_A;
	// float tau_q = 0.0004f/0.72f; //C_LS_A/C_RS_A;
	// float Inv_RS_A = 1.f/0.72f; //C_RS_A;

	float tau_d = 0.0002349f/0.3774f; //C_LS_A/C_RS_A;
	float tau_q = 0.0002349f/0.3774f; //C_LS_A/C_RS_A;
	float Inv_RS_A = 1.f/0.3774f; //C_RS_A;

	machine_obs1 = ComplexVar(Inv_RS_A*(1 - exp(-C_SAMPLE_TIME/tau_d)),Inv_RS_A*(1 - exp(-C_SAMPLE_TIME/tau_q)));
	machine_obs2 = ComplexVar(exp(-C_SAMPLE_TIME/tau_d),exp(-C_SAMPLE_TIME/tau_q));	

	Ldq = ComplexVar(C_LS_A, C_LS_A);
	lambda_pm = ComplexVar(C_LAMBDA_PM_B, 0);
}
void  	CurrentObserver::update(const ComplexVar& Idq_k, const ComplexVar& v_qd_f)
{
	I_dq = Idq_k;
	v_qd_f_com = v_qd_f;
	run();
}
void  	CurrentObserver::clear()
{
		I_dq_err.clear();
		I_dq_f_hat_kp1.clear();
		I_dq_c_hat_kp1.clear(); 
		I_dq_f_hat.clear();
		I_dq_c_hat.clear();
		I_dq.clear();
		v_qd_f_com.clear();
		v_qd_c_com.clear();
		I_dq_err_km1.clear();
		I_dq_err_km2.clear();
		M_r_dq_or_k.clear();
		M_r_dq_or_km1.clear();
		M_r_dq_or_km2.clear();
		M_p_dq_or.clear();
		M_i_dq_or.clear();
		M_decoupl_dq.clear();
		M_dq_or.clear();
		M_decoupl_d = 0.f, M_decoupl_q = 0.f;
		w_r = 0.f;	
}
void  	CurrentObserver::run()
{			
	I_dq_err = I_dq - I_dq_f_hat;// - I_dq_c_hat;

	M_p_dq_or  = I_dq_err.comp_mul(Kco);
	M_i_dq_or += I_dq_err.comp_mul(Kico);
	//M_r_dq_or_k = I_dq_err_km1.comp_mul(Kro) - I_dq_err_km2.comp_mul(Kro)*(1.1f) +  M_r_dq_or_km1*C_COSWCTS2- M_r_dq_or_km2;
	
	//M_decoupl_dq =   ComplexVar(0,1)*(Ldq.comp_mul(I_dq_f_hat_kp1) + lambda_pm)*(-w_r); 		

	M_dq_or = M_p_dq_or + M_i_dq_or + v_qd_f_com ;//+ M_decoupl_dq;
		
    E_dq_est = M_p_dq_or + M_i_dq_or;
    E_abs_inv = sqrt(E_dq_est.get_d_comp()*E_dq_est.get_d_comp() + E_dq_est.get_q_comp() * E_dq_est.get_q_comp());

     ComplexVar E_norm(1/E_abs_inv,1/E_abs_inv);
   // E_norm_dq_est = E_dq_est.comp_mul(E_norm);
     E_norm_dq_est = E_dq_est;
    
	I_dq_f_hat_kp1 = machine_obs1.comp_mul(M_dq_or)  + machine_obs2.comp_mul(I_dq_f_hat);
	//I_dq_c_hat_kp1 = machine_obs1.comp_mul(M_r_dq_or_k + v_qd_c_com) + machine_obs2.comp_mul(I_dq_c_hat);
	
			
	I_dq_f_hat = I_dq_f_hat_kp1;				//Set up for next sample period
	//I_dq_c_hat = I_dq_c_hat_kp1;				//Set up for next sample period

	//I_dq_err_km2 = I_dq_err_km1;
	//I_dq_err_km1 = I_dq_err;
	//M_r_dq_or_km2 = M_r_dq_or_km1;
	//M_r_dq_or_km1 = M_r_dq_or_k;

}
ComplexVar 	CurrentObserver::get_I_dq_f_hat_kp1()
{
	return I_dq_f_hat_kp1;
}
/*ComplexVar 	CurrentObserver::get_I_dq_c_hat_kp1()
{
	return I_dq_c_hat_kp1;
}*/

float 	CurrentObserver::get_E_norm_d_est()
{
	return E_norm_dq_est.get_d_comp();
}
float 	CurrentObserver::get_E_norm_q_est()
{
	return E_norm_dq_est.get_q_comp();
}

float 	CurrentObserver::get_E_d_est()
{
	return E_dq_est.get_d_comp();
}
float 	CurrentObserver::get_E_q_est()
{
	return E_dq_est.get_q_comp();
}



