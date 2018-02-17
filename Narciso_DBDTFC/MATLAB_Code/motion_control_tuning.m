Jp = 1.9e-5;

Ts = 1/2000;

p1 = exp(-2*pi*Ts*2);
p2 = exp(-2*pi*Ts*5);
p3 = exp(-2*pi*Ts*50);

ba = Jp/Ts*(1-p1*p2*p3)
Ksa = Jp/Ts^2*(3-p1*p2-p2*p3-p1*p3)-2*ba/Ts
Kisa = ((3-p1-p2-p3)*Jp-Ts*ba-Ts^2*Ksa)/Ts^3

z = tf('z',Ts);

T = Ts*z*(ba-(Ts*Ksa+2*ba)*z+(Ts^2*Kisa+Ts*Ksa+ba)*z^2)/...
    (Jp*z^3+(-3*Jp+Ts*ba+Ts^2*Ksa+Ts^3*Kisa)*z^2+...
    (3*Jp-2*Ts*ba-Ts^2*Ksa)*z+Ts*ba-Jp);

p = bodeoptions;
p.FreqUnits = 'Hz';
p.MagUnits = 'abs';
p.MagScale = 'linear';

figure(1), bode(T,p), grid on, hold on;