//Back-EMF Observer
//04-09 Revised by RZ

#pragma once
#include "constants.h"
#include "ComplexVar.h"



class CurrentObserver 
{
	private:
		// Member functions
		void  	run();
		ComplexVar Kro, Kco, Kico;
		ComplexVar I_dq_err, I_dq_f_hat_kp1,I_dq_c_hat_kp1, I_dq_f_hat, I_dq_c_hat, I_dq, v_qd_f_com, v_qd_c_com;
		ComplexVar I_dq_err_km1,I_dq_err_km2, M_r_dq_or_k, M_r_dq_or_km1, M_r_dq_or_km2;
		ComplexVar M_p_dq_or, M_i_dq_or, M_decoupl_dq, M_dq_or;
		ComplexVar machine_obs1, machine_obs2, Ldq, lambda_pm;
		ComplexVar E_dq_est, E_norm_dq_est;
		float M_decoupl_d, M_decoupl_q;
		float w_r;
	    float E_abs_inv;
	public:
		CurrentObserver();
		void 	clear();
		void  	update(const ComplexVar& Idq_k, const ComplexVar& v_qd_f);
		
				
		float   get_E_norm_q_est();
		float   get_E_norm_d_est();
		
	    float   get_E_q_est();
		float   get_E_d_est();


		
		ComplexVar 	get_I_dq_f_hat_kp1();
	//	ComplexVar 	get_I_dq_c_hat_kp1();

};
