a = 2*pi*280;
w = 2*pi*10;
Ts = 1/20000;

% del1 = 2*exp(-a*Ts)*cos(w*Ts);
% del2 = exp(-2*a*Ts);
% alpha = exp(-a*Ts)*(a*sin(w*Ts)-w*cos(w*Ts));

s = tf('s');
z = tf('z',Ts);
% syms s;

% T = (a^2+w^2)/((s+a)^2+w^2);
% Tz = -((1-z^(-1))/(a^2+w^2))*(w+z^(-1)*alpha)/...
%     (1-del1*z^(-1)+del2*z^(-2));
% Tz = minreal(Tz);
% 
% H = -exp(-s*Ts)*(1-exp(-s*Ts))*T*(1-del1*exp(-s*Ts)+del2*exp(-2*s*Ts))...
%      /(w+alpha*exp(-s*Ts))*(a^2+w^2);
% 
% H = minreal(H);
% 
% % pretty(H)
% %  impulse(T), hold on;
% %  impulse(Tz);
%  step(H);

%%

del1 = exp(-a*Ts);

H = (1-exp(-s*Ts))/s*(1-del1*exp(-s*Ts))/(1-del1)*a/(s+a)/Ts;

Y = H/s;
U = 0.43/(1-del1)*(1/s-Y)*(1-del1*exp(-s*Ts))/Ts;
H = minreal(H);

impulse(1/s), hold on;
step(U), hold on;
step(H);

% t = 0:Ts/10000:0.001;
% 
% y = 1/(1-del1)/Ts*(t-Ts-(1-exp(-a*(t-Ts)))/a-...
%     (1+del1)*(t-2*Ts-(1-exp(-a*(t-2*Ts)))/a)...
%     +del1*(t-3*Ts-(1-exp(-a*(t-3*Ts)))/a));
% 
% plot(t,y), ...
%     ylim([0 1.2]);