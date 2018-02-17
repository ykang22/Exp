%% FLUX OBSERVER TUNING
% THIS CODE TUNES THE FLUX OBSERVER BY REDUCING THE OBSERVER TO A FIRST-
% ORDER OBSERVER BASED ON A DESIRED BREAK FREQUENCY

close all; clear all; clc;

fc = 1000; % Hz
T = 1/10000; % Sample Period
zp1 = exp(-2*pi*fc*T);
zp2 = exp(-2*pi*10*T);

%phi = 1/T*(1+sqrt((zp+1)^2+1));
K1 = (1-zp1*zp2)/T; %1/(zp/phi+T);
K2 = (2-T*K1-zp1-zp2)/T^2; %1/T*(K1-phi);

z = tf('z',T);

H = T*((K1+T*K2)*z-K1)/(z^2+z*(T*K1+T^2*K2-2)+(1-K1*T));

p = bodeoptions;
p.FreqUnits = 'Hz';
p.MagUnits = 'abs';
p.MagScale = 'linear';

bode(H,p), grid on;