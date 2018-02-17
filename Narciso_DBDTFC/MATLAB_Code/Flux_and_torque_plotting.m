lambda_qs_curr = data1(2:end,1);
lambda_ds_curr = data1(2:end,2);
lambda_qs_volt = data1(2:end,3);
lambda_ds_volt = data1(2:end,4);

T_star = data1(:,10);
T_hat = data1(:,11);
T_hat_curr = data1(:,9)*3/2*4*0.0067197;

t = (0:length(T_hat)-1)/20000;

figure(1),plot(t,T_star), hold on;
figure(1),plot(t,T_hat);
figure(1),plot(t,T_hat_curr);

figure(2), plot(lambda_qs_curr,lambda_ds_curr), axis equal, hold on;
figure(2), plot(lambda_qs_volt,lambda_ds_volt), grid on;

% figure(4), plot(t(2:end),lambda_qs_curr-lambda_qs_volt), hold on;
% figure(4), plot(t(2:end),lambda_ds_curr-lambda_ds_volt);

figure(4), plot(t(2:end),lambda_qs_curr,...
    'LineWidth',1.5), hold on;
figure(4), plot(t(2:end),lambda_qs_volt,...
    'LineWidth',1.5), hold on;
figure(4), plot(t(2:end),lambda_ds_curr,...
    'LineWidth',1.5), hold on;
figure(4), plot(t(2:end),lambda_ds_volt,...
    'LineWidth',1.5), hold on ,grid on;
