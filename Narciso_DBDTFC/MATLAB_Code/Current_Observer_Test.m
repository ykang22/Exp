%% CURRENT OBSERVER TEST
% THIS CODE TUNES THE CURRENT OBSERVER UTILIZED FOR DB-DTFC UTILIZING
% AN EXPLICIT SOLUTION METHOD: TO RENDER THE SYSTEM A FIRST ORDER SYSTEM,
% THE POLYNOMIAL EQUATION'S DETERMINANT IS SET TO ZERO, PRODUCING A
% CONIC SECTION WHERE THE GAINS ARE ITS PARAMETERS. THE INTERSECT OF THE
% LATTER CURVE AND A LINE AT A DESIRED FREQUENCY IS THE SOLUTION TO THE 
% GAINS.

close all; clear all; clc;

%%
T = 1/10000;
R = 0.7/sqrt(3);
L = 0.4e-3/sqrt(3);
tau = L/R;
del = (1-exp(-T/tau))/R;
gamma = exp(-T/tau);
fcc = 500; % Cut-off frequency: fcc = (gamma+del)/2/tau
zpp = exp(-2*pi*fcc*T);

cc = gamma+zpp^2-(gamma+1)*zpp;
A = 0.5/del*(-(2*zpp-gamma-1)+sqrt((2*zpp-gamma-1)^2-4*cc));
K3 = gamma*A/(zpp+A*del);
K4 = K3/T*(1/A-1);


z = tf('z',T);

H = del*((K3+K4*T)*z-K3)/...
    ((z^2+(del*K3+del*K4*T-gamma-1)*z+gamma-del*K3));
p = bodeoptions;
p.FreqUnits = 'Hz';
p.MagUnits = 'abs';
p.MagScale = 'linear';

[mag,pha,fout,sdmag,sdpha] = bode(H,p);
mag = squeeze(mag);
pha = squeeze(pha);
fout = fout/(2*pi);

% %% THIS PART IS SUPER FUZZY, NOT IN THE GOOD WAY
% fc = 10;
% zp = exp(-2*pi*fc*T);
% 
% a = (4*del-2*(1+gamma)*del);
% b = -2*(1+gamma)*T*del;
% c = 2*del^2*T;
% d = (del*T)^2;
% e = del^2;
% f = -(gamma-1)^2;
% 
% intrsct = -(2*zp-1-gamma)/del/T;
% slope = -1/T;
% 
% syms x y
% eqn1 = slope*x+intrsct-y == 0;
% eqn2 = a*x+b*y+c*x*y+d*y^2+e*x^2-f == 0;
% 
% [solx, soly] = solve(eqn1, eqn2);
% 
% % fh = @(x,y) a*x+b*y+c*x*y+d*y^2+e*x^2-f;
% % %fg = @(x,y) 0.5*(1+gamma-del*x-del*T*y)-zp;
% % figure(2), ezplot(fh,[-50,50,0,1000]), hold on;
% % %figure(2), ezplot(fg, [-10,0,-10e7,100]), hold on;
% % axis equal
% % grid on

%%
f0 = 0;
ffinal = 1/T/2;
tsweep = 10;
m = (ffinal-f0)/tsweep;
n = 10*tsweep/T;

ierr = 0;
ihat = 0;
ierracc = 0;
iq = zeros(1,n);
phase = zeros(1,n+1);
ikp1 = zeros(1,n+1);

for i = 1:n
    iq(i) = 5*sin(2*pi*i*T*(f0+m/2*i*T));
    ihat = ikp1(i);
    ierr = iq(i)-ihat;
    ierracc = ierracc+T*ierr;

    vcur = K3*ierr+K4*ierracc;

    ikp1(i+1) = gamma*ihat+del*vcur;
end

ikp1(end) = [];
figure(2), plot(iq), hold on;
figure(2), plot(ikp1);

[Txy,Ft] = tfestimate(iq,ikp1,rectwin(512),256,2^14,1/T);
[Cxy,F] = mscohere(iq,ikp1,rectwin(512),256,2^14,1/T);

figure(3), subplot(2,1,1), semilogx(Ft,abs(Txy),'b'), grid on, xlim([1,1e4]),...
    ylim([0,1.2]), hold on, semilogx(fout,mag,'r');

figure(3), subplot(2,1,2), semilogx(F,angle(Txy)*180/pi,'b'), grid on,...
    xlim([1,1e4]), hold on, semilogx(fout,pha,'r');

figure(4), semilogx(F,Cxy), xlim([1,1e4]), ylim([0,1]),grid on;



