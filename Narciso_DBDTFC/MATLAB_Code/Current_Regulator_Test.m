%==========================================================================
%% CURRENT REGULATOR SIMULINK SIGNAL PROCESSING CODE

t = id_sim.time;
id = id_sim.Data;
iq = iq_sim.Data;
iqd = iq - 1i*id;
iq_hat = iq_hat.Data;
id_hat = id_hat.Data;
iqd_hat = iq_hat - 1i*id_hat;

% del_theta = del_theta.Data;
% del_theta_star = squeeze(del_theta_star.Data);

id_star = squeeze(id_star_sim.Data);
iq_star = squeeze(iq_star_sim.Data);
iqd_star = iq_star - 1i*id_star;

%% CURRENT FRF
[Txy,Ft] = tfestimate(iqd,iqd_hat,rectwin(256),128,2^12,1/Ts,'centered');
[Cxy,F] = mscohere(iqd,iqd_hat,rectwin(256),128,2^12,1/Ts,'centered');

i0 = find(Ft == 0);
Ft_neg = -Ft(1:i0);

figure(3), subplot(3,2,1), semilogx(Ft_neg,abs(Txy(1:i0))), ...
    grid on, ylim([0,1.2]);
figure(3), subplot(3,2,2), semilogx(Ft(i0:end),abs(Txy(i0:end))), grid on,...
    ylim([0,1.2]);

figure(3), subplot(3,2,3), semilogx(Ft_neg,angle(Txy(1:i0))*180/pi), grid on;
    ylim([-180,180]);
figure(3), subplot(3,2,4), semilogx(Ft(i0:end),angle(Txy(i0:end))*180/pi), grid on;
    ylim([-180,180]);
    
figure(3), subplot(3,2,5), semilogx(Ft_neg,Cxy(1:i0)), ylim([0,1]),grid on;
figure(3), subplot(3,2,6), semilogx(Ft(i0:end),Cxy(i0:end)), ylim([0,1]),grid on;

%% MOTION FRF
% [Txy, Ft] = tfestimate(del_theta_star,del_theta,rectwin(256),128,2^10,...
%     1/Ts_motion,'onesided');
% figure(4), semilogx(Ft,abs(Txy));
% figure(5), semilogx(Ft,angle(Txy)*180/pi);