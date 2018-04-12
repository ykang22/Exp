
clear all
load ME577_inductance.mat



clear i1 i2 i3
theta=0
for n=1:length(J_a)

i1=J_a(n)*ampere_per_density;
i3=J_b(n)*ampere_per_density;
i2=J_c(n)*ampere_per_density;

idq = abc2dq(i1,i2,i3,theta*pi/180);

i_dd(n)=idq(1);
i_qq(n)=idq(2);
% 
flux11=-flux_phase_b(n)*0.5;
flux21=-flux_phase_c(n)*0.5;
flux31=-flux_phase_a(n)*0.5;
fluxdq1 = abc2dq(flux11,flux21,flux31,theta*pi/180);

fluxd(n)=fluxdq1(1);
fluxq(n)=fluxdq1(2);

end

num_iter=21
for p=1:1:num_iter
    for q=1:1:num_iter
    i_d(p,q)=i_dd((p-1)*num_iter+q);
    i_q(p,q)=i_qq((p-1)*num_iter+q);
    flux_q(p,q)=fluxq((p-1)*num_iter+q);
    flux_d(p,q)=fluxd((p-1)*num_iter+q);
    end
end

for p=1:1:size(flux_d,1)
    diff_flux_d_d(p,:)=diff(flux_d(p,:))./diff(i_d(p,:))*1000;
    diff_flux_q_q(:,p)=diff(flux_q(:,p))./diff(i_q(:,p))*1000;    
    diff_flux_d_q(p,:)=diff(flux_d(:,p))./diff(i_q(:,p))*1000;
    diff_flux_q_d(:,p)=diff(flux_q(p,:))./diff(i_d(p,:))*1000; 
end

id_step=diff(i_d(1,1:2))
iq_step=diff(i_q(1:2,1))
[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = flux_d%-flux_d(11,11);
figure(111)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('\lambda_d[Vs]')
% ylim([-5,5])
% xlim([-5 5])
% zlim([-0.01 0.01])
grid on
% hold on

[X,Y] = meshgrid(-10*iq_step:iq_step:10*iq_step,-10*id_step:id_step:10*id_step);
Z = flux_q;
figure(112)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('\lambda_q[Vs]')
% ylim([-5, 5])
% zlim([-0.01 0.01])
grid on

[X,Y] = meshgrid(-9.5*id_step:id_step:9.5*id_step,-10*iq_step:iq_step:10*iq_step);
Z = diff_flux_d_d;
figure(114)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_d_d[mH]')
zlim([0 0.3])
grid on
% title('Variation of q-axis cross-coupling inductance L_q_d')

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = diff_flux_d_q;
figure(117)
s = surf(X,Y,Z);
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_d_q[mH]')
zlim([-0.1 0.1])
grid on

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = diff_flux_q_q;
figure(115)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_q_q[mH]')
zlim([0 0.3])
grid on
view(1)
[X,Y] = meshgrid(-9.5*id_step:id_step:9.5*id_step,-10*iq_step:iq_step:10*iq_step);
Z = diff_flux_q_d;
figure(116)
s = surf(X,Y,Z');
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_q_d[mH]')
zlim([-0.1 0.1])
grid on
