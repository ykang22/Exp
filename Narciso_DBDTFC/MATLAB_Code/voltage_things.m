% plot(data1(:,1),data1(:,2)),axis equal, grid on, hold on;

vab = detrend(data1(:,8));%./data1(:,11);
vca = detrend(data1(:,9));%./data1(:,11);
vbc = detrend(data1(:,10));%./data1(:,11);

va1 = 1/3*(vab-vca);
vb1 = -1/3*(2*vab+vca);
vc1 = 1/3*(vab+2*vca);
% 
% va2 = -1/3*(2*vca+vbc);
% vb2 = 1/3*(vca+2*vbc);
% vc2 = 1/3*(vca-vbc);
% 
% va = va1+va2;
% vb = vb1+vb2;
% vc = vc1+vc2;

vq = 2/3*(vab-1/2*(vbc+vca));
vd = -1/sqrt(3)*(vbc-vca);

figure(100),plot(vq,vd),axis equal, grid on;