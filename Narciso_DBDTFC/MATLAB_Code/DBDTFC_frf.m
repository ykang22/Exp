%% TORQUE

T_star = (data1(:,15));
T_hat = (data1(:,16));
% % T_c = detrend(data1(:,9)*3/2*4*0.0067);
% 
% 
% 
% [H,fh] = mscohere(T_star,T_hat,rectwin(1000),100,2^16,20000,'onesided');
% [T,f] = tfestimate(T_star,T_hat,rectwin(1000),100,2^16,20000,'onesided');
% % [Hw,fhw] = mscohere(T_star,T_c,rectwin(1000),100,2^16,20000,'onesided');
% % [W,fw] = tfestimate(T_star,T_c,rectwin(1000),100,2^16,20000,'onesided');
% 
% 
% figure(1), semilogx(f,abs(T),'LineWidth',2),...
%     ylim([0 1.25]), xlim([min(f) max(f)]), grid on, hold on;
% %figure(1), semilogx(fw,abs(W),'LineWidth',2);
% figure(2), semilogx(f,180/pi*unwrap(angle(T)),'LineWidth',2),...
%     xlim([min(f) max(f)]), grid on, hold on;
% %figure(2), semilogx(fw,180/pi*angle(W),'LineWidth',2);
% figure(3), semilogx(fh,H,'LineWidth',2), ylim([0 1]),...
%     xlim([min(f) max(f)]), grid on, hold on;
% %figure(3), semilogx(fhw,Hw,'LineWidth',2);

t_T = (0:length(T_star)-1)/20000;

figure(8), plot(t_T, T_star), hold on;
figure(8), plot(t_T, T_hat), grid on;

%% FLUX

% load('DB-DTFC_CHIRP_000_FLUX_TEST.mat');

lambda_star = (data1(:,2)*0.22e-3*0.5)+0.0067197;
lambda_hat = data1(:,14);%sqrt((data1(:,3)).^2+(data1(:,4)).^2);

t = (0:length(lambda_star)-1)/20000;

% [L,fl] = tfestimate(lambda_star,lambda_hat,rectwin(1000),...
%     100,2^16,20000,'onesided');
% [H,fh] = mscohere(lambda_star,lambda_hat,rectwin(1000)...
%     ,100,2^16,20000,'onesided');
% 
% figure(4), hold on, semilogx(fl,abs(L),'LineWidth',1.5)...
%     ,grid on, xlim([min(fl) max(fl)]),...
%     ylim([0 2]);
% figure(5), hold on, semilogx(fl,180/pi*(angle(L)),'LineWidth',1.5),...
%     xlim([min(fl) max(fl)]), ylim([-90 90]), grid on;
% figure(6), hold on, semilogx(fh,H,'LineWidth',1.5),xlim([min(fl) max(fl)]),...
%     ylim([0 1]), grid on;

figure(7), plot(t, lambda_star), hold on;
figure(7), plot(t, lambda_hat), grid on;


