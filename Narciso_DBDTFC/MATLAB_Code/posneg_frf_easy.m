pos = false;

iq_star = data1(2:end,1);
id_star = data1(2:end,2);
iq = (data1(2:end,5));
id = data1(2:end,6);

fs = 20000;

[T,f] = tfestimate(iq_star-1i*id_star,iq-1i*id,kaiser(1000,5),...
    750,2^12,fs,'centered');
[C,fc] = mscohere(iq_star-1i*id_star,iq-1i*id,kaiser(1000,5),...
    750,2^12,fs,'centered');

if pos
k = ceil(length(f)/2):length(f);
figure(1), semilogx(f(k),abs(T(k))), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
figure(2), semilogx(f(k),unwrap(angle(T(k)))*180/pi), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
figure(3), semilogx(fc(k),C(k)), grid on, hold on,...
    xlim([min(f(k)) max(f(k))]);
else
h = 1:ceil(length(f)/2)-1;
figure(4), semilogx(f(h),abs(T(h))), grid on, hold on,...
    xlim([min(f(h))-5 max(f(h))]);
figure(5), semilogx(f(h),unwrap(angle(T(h)))*180/pi+720), grid on,...
    hold on,xlim([min(f(h))-5 max(f(h))]);
figure(6), semilogx(fc(h),C(h)), grid on, hold on,...
    xlim([min(f(h))-5 max(f(h))]);
end