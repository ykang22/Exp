R = 0.43665;
L = (0.00015574 + 0.000152114)/2;
tau = L/R;
T = 1/20000;
n = 100/T;

f1 = 1;
f2 = 100;%1/tau/2/pi;
p1 = exp(-T*2*pi*f1);
p2 = exp(-T*2*pi*f2);

k1 = (1-p1*p2)/T;
k2 = 1/T^2*(2-p1-p2-T*k1);

% k1 = 5.437766721169132e+03;%6.793445466736102e2;
% k2 = 3.382543631774926e+05;%3.880375062195185e+04;

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

% FLUX OBSERVER WITHOUT CFF
for i = 1:n+1
    l_err = y(i)-l_kp1;
    l_err_acc = l_err_acc + T*l_err;
    
    b_k = T*(k1*l_err+k2*l_err_acc);
    l_kp1 = l_kp1 + b_km1;
    b_km1 = b_k;
    
    l_out(i) = l_kp1;
end

[T,f] = tfestimate(y,l_out,rectwin(40000),500,2^16,20000,'onesided');
[H,fh] = mscohere(y,l_out,rectwin(40000),500,2^16,20000,'onesided');
figure(1),semilogx(f,abs(T));
figure(2),semilogx(f,180/pi*angle(T));
figure(3), semilogx(fh,H);

% T = 1/20000;
% z = tf('z',T);
% V = T*z*(z-1)/(z^2+(k1*T+T^2*k2-2)*z+1-T*k1);
% I = R*T/2*(z^2-1)/(z^2+(k1*T+T^2*k2-2)*z+1-T*k1);
% Vcff = V-I;
% p = bodeoptions;
% p.FreqUnits = 'Hz';
% p.MagUnits = 'abs';
% p.MagScale = 'linear';
% bode(V,p), hold on;
% bode(I,p), hold on;
% bode(Vcff,p)