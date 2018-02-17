T = 1/20000;
f1 = 1;
f2 = 100;
p1 = exp(-T*2*pi*f1);
p2 = exp(-T*2*pi*f2);

k1 = (1-p1*p2)/T;
k2 = 1/T^2*(2-p1-p2-T*k1);

z = tf('z',T);
H = z*T*(z*(T*k2+k1)-k1)/(z^2+z*((T*k2+k1)*T-2)+1-T*k1);

p = bodeoptions;
p.FreqUnits = 'Hz';
p.MagUnits = 'abs';
p.MagScale = 'linear';

figure(1),bode(H,p), grid on;