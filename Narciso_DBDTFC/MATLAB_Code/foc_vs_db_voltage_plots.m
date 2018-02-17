vqefoc = data1(:,5);
vdefoc = data1(:,6);
vqedb  = data1(:,7);
vdedb  = data1(:,8);

theta_comp = data1(:,12);

t = (0:length(vqefoc)-1)/20000;

vqsfoc = vqefoc.*cos(theta_comp)+vdefoc.*sin(theta_comp);
vdsfoc = -vqefoc.*sin(theta_comp)+vdefoc.*cos(theta_comp);

vqsdb = vqedb.*cos(theta_comp)+vdedb.*sin(theta_comp);
vdsdb = -vqedb.*sin(theta_comp)+vdedb.*cos(theta_comp);

figure(2), plot(vqsfoc(2:end)/20000,vdsfoc(2:end)/20000), hold on, axis equal;
figure(2), plot(vqsdb(2:end)/20000,vdsdb(2:end/20000)), grid on;

% figure(2), plot(t,vqsfoc,'LineWidth',1.5), hold on;
% figure(2), plot(t,vqsdb,'LineWidth',1.5);
% figure(2), plot(t,vdsfoc,'LineWidth',1.5);
% figure(2), plot(t,vdsdb,'LineWidth',1.5), grid on;
% 
% figure(3), plot(t,vqefoc,'LineWidth',1.5), hold on;
% figure(3), plot(t,vqedb,'LineWidth',1.5);
% figure(3), plot(t,vdefoc,'LineWidth',1.5);
% figure(3), plot(t,vdedb,'LineWidth',1.5), grid on;