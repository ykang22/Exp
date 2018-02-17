vq = detrend(data1(2:end,15));
vd = data1(2:end,16);
iq = detrend(data1(2:end,9));
id = detrend(data1(2:end,10));

[T1,f1] = tfestimate(vq,iq,kaiser(1000,5),750,2^12,20000);
[H1,fh1] = mscohere(vq,iq,kaiser(1000,5),750,2^12,20000);

figure(2), semilogx(f1,angle(T1)*180/pi),hold on;
figure(100), semilogx(f1,abs(2*pi*f1*1i.*T1)),hold on;
figure(3), semilogx(fh1,H1), hold on;