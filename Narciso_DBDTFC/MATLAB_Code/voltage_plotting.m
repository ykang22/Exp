%%
% FROM LINE-TO-LINE VOLTAGES TO DQ

vab = data1(:,4)+29.8983383;
vca = data1(:,5)+29.3381710;
vbc = data1(:,6)+29.6507282;

theta = 4*data1(:,3);
% omega = data1(:,10);
T = 1/20000;

theta_comp = theta;% + 0.8*omega*T;

va1 = 1/3*(vab-vca);
vb1 = -1/3*(2*vab+vca);
vc1 = 1/3*(vab+2*vca);

va2 = -1/3*(2*vca+vbc);
vb2 = 1/3*(vca+2*vbc);
vc2 = 1/3*(vca+vbc);

vq1 = 2/3*(va1-(vb1+vc1)/2);
vd1 = sqrt(3)/3*(vc1-vb1);

vq2 = 2/3*(va2-(vb2+vc2)/2);
vd2 = sqrt(3)/3*(vc2-vb2);

vqe = vq1.*cos(theta_comp)+vd1.*sin(theta_comp);
vde = -vq1.*sin(theta_comp)+vd1.*cos(theta_comp);

figure(1), plot(vqe), hold on,
plot(vde);

figure(2), plot(vq1), hold on,
plot(vd1);

figure(3), plot(vq1,vd1), axis equal;

figure(4), plot(va1), hold on, plot(vb1), hold on, plot(vc1);

figure(5), plot(vab), hold on, plot(vca), hold on, plot(vbc);