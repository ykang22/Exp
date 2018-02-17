%%
iqmax = 5;
idmax = 5;
lambda = 0.0065;
Ts = 1/10000;
theta = 0;
omega = 200;
iq_int_err = 0;
id_int_err = 0;



theta = theta + omega*Ts;

%%
theta_r_2 = wrapTo2Pi(4*theta);
if(ids_e_star > idmax)
    ids_e_star = idmax;
end

% iqs_e_star = tem_star/12/lambda;
if(iqs_e_star > iqmax)
    iqs_e_star = iqmax;
end

%%
% iqds_s = iqs_s-1i*ids_s;
% iqds_e = iqds_s*exp(-theta_r_2);
% iqs_e = real(iqds_e);
% ids_e = -imag(iqds_e);

%%
iq_error = iqs_e_star - iqs_e;
id_error = ids_e_star - ids_e;

iq_int_error = iq_int_error + iq_error*Ts;
id_int_error = id_int_error + id_error*Ts;

vqs_e_star = iq_error*Kp + Ki*iq_int_error;
vds_e_star = id_error*Kp + Ki*id_int_error;

vqs_e_star = vqs_e_star + id_int_err*Kp*omega + ...
    lambda*omega;
vds_e_star = vds_e_star - iq_int_err*Kp*omega;

%%
vqds_e_star = vqs_e_star - 1i*vds_e_star;
% vqds_s_star = vqds_e_star*exp(theta_r_2);



