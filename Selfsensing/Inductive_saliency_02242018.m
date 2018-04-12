clear all

%No excitation
clc
R_u=1.1;
R_v=1.0;
R_w=1.0;
N=1;

L_qq= N^2*(4*R_u+R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dq= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_qd= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dd= 3*N^2*(R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))

%%
clc
%flux intensifying
R_u=1.5;
R_v=1.05;
R_w=1.05;
N=1;

L_qq= N^2*(4*R_u+R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dq= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_qd= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dd= 3*N^2*(R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))

%%
clc
%flux weakening
R_u=1.0;
R_v=1.0;
R_w=1.0;
N=1;

L_qq= N^2*(4*R_u+R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dq= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_qd= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dd= 3*N^2*(R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))




%%
clc
%Torque
R_u=1.1;
R_v=1.1;
R_w=0.9;
N=1;

L_qq= N^2*(4*R_u+R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dq= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_qd= 3^0.5*N^2*(R_v-R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))
L_dd= 3*N^2*(R_v+R_w)/(2*(R_u*R_v+R_u*R_w+R_v*R_w))

