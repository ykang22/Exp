close all; clear all; clc;
pha_km2 = 0;
pha_km1 = 0;
pha_k = 0;

T = 1/20000;
f0 = 4000;
f1 = 8000;
T_span = 0.5;
slope = (f1-f0)/T_span;

k = 0;
t_tot = 2;

time = zeros(0,t_tot/T-1);
y = zeros(0,t_tot/T-1);
i = 1;

tic
for k = 1:t_tot/T
    time(k) = k*T;
    i = mod(k,T_span/T);
    y(k) = 2*cos(2*pi*i*T*(f0+slope/2*i*T));
end
toc

% pha_max = f0*T_span+slope/2*T_span^2;
% tic
% for k = 1:t_tot/T
%     time(k) = k*T;
%     y(k) = 2*cos(2*pi*pha_k);
%     pha_k = 2*pha_km1-pha_km2+slope*T^2;
%     pha_k = mod(pha_k,pha_max);
%     pha_km2 = mod(pha_km1,pha_max);
%     pha_km1 = mod(pha_k,pha_max);
% end
% toc

spectrogram(y,kaiser(256,5),128,2^12,1/T);