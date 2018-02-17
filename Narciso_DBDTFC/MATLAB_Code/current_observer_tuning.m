R = 0.43665;
L = 0.0002202481797808901;%(0.00015574 + 0.000152114)/2;
tau = L/R;
T = 1/20000;
n = 100/T;

f1 = 200;%1/tau/2/pi;
f2 = 1/tau/2/pi;%4.514802854669817e+02;
p1 = exp(-T*2*pi*f1);
p2 = exp(-T*2*pi*f2);

gam = exp(-T/tau);
del = 1-gam;

k3 = R/del*(gam-p1*p2);%0.77249721492451;%
k4 = 1/T*(R/del*(gam+1-p1-p2)-k3);%2354.393299072175;

% zero = k3/(k3+T*k4);
% szero = -1/T*log(zero);
% 
% z = tf('z',T);
% H = z*(z-zero)*del/R*(R/del*(gam-p1*p2)+T*1/T*(R/del*(gam+1-p1-p2)-k3))/(z-p1)/(z-p2);
% J = z*del/R*(z*(k3+T*k4)-k3)/(z^2+(del/R*(k3+T*k4)-(gam+1))*z...
%     +gam-k3*del/R);
% O = z*del/R*(z*(k3+T*k4)-k3)/(z-1)/(z-gam);
% % figure(2),nyquist(O);
% p = bodeoptions;
% p.FreqUnits = 'Hz';
% p.MagUnits = 'abs';
% p.MagScale = 'linear';
% figure(1),bode(J,p), hold on;
% % figure(1),bode(H)

disp(k3)
disp(k4)

f0 = 0;
f1 = 10000;
T_span = 10;
slope = (f1-f0)/T_span;

k = 0;
i = 1;
% GENERATE CHIRP
for k = 1:n+1
    time(k) = k*T;
    i = mod(k,T_span/T);
    y(k) = 2*cos(2*pi*i*T*(f0+slope/2*i*T));
end

l_kp1 = 0;
l_err_acc = 0;
l_err = 0;
b_km1 = 0;

% CURRENT OBSERVER WITHOUT CFF
for i = 1:n+1
    l_err = y(i)-l_kp1;
    l_err_acc = l_err_acc + T*l_err;
    
    b_k = 1/R*del*(k3*l_err+k4*l_err_acc);
    l_kp1 = gam*l_kp1 + b_km1;
    b_km1 = b_k;
    
    l_out(i) = l_kp1;
end

[T,f] = tfestimate(y,l_out,rectwin(40000),500,2^16,20000,'onesided');
[H,fh] = mscohere(y,l_out,rectwin(40000),500,2^16,20000,'onesided');
figure(1), semilogx(f,abs(T));
figure(2), semilogx(f,180/pi*angle(T));
figure(3), semilogx(fh,H);