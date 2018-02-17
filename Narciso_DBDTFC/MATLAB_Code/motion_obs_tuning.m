Jp = 1.9e-5;

Ts = 1/2000;

p1 = exp(-2*pi*Ts*1);
p2 = exp(-2*pi*Ts*5);
p3 = exp(-2*pi*Ts*50);

pj = p1+p2+p3;
pij = p1*p2+p2*p3+p1*p3;
pijk = p1*p2*p3;

bo = 2*Jp/Ts*(7-(pj+pij+pijk))/(pijk+pij+pj+1)
Kso = (pijk*(2*Jp+Ts*bo)+Ts*bo-2*Jp)/Ts^2
Kiso = (pij*(2*Jp+Ts*bo)+Ts*bo-6*Jp)/Ts^3