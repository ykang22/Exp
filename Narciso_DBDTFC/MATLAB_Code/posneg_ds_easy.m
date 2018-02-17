pos = true;

vq_star = data1(2:end,14);
vd_star = data1(2:end,15);
iq = detrend(data1(2:end,5));
id = detrend(data1(2:end,6));

fs = 20000;

[T,f] = tfestimate(iq-1i*id,vq_star-1i*vd_star,kaiser(10000,5),...
    750,2^12,fs,'centered');
[C,fc] = mscohere(iq-1i*id,vq_star-1i*vd_star,kaiser(10000,5),...
    750,2^12,fs,'centered');

if pos
k = ceil(length(f)/2):length(f);
figure(10), loglog(f(k),abs(T(k))), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
figure(20), semilogx(f(k),(angle(T(k)))*180/pi), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
figure(30), semilogx(fc(k),C(k)), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
else
h = 1:ceil(length(f)/2)-1;
figure(40), loglog(f(h),abs(T(h))), grid on, hold on,...
    xlim([min(f(h))-5 max(f(h))]);
figure(50), semilogx(f(h),(angle(T(h)))*180/pi), grid on,...
    hold on,xlim([min(f(h))-5 max(f(h))]);
figure(60), semilogx(fc(h),C(h)), grid on, hold on,...
    xlim([min(f(h))-5 max(f(h))]);
end