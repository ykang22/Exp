function fft_gen(v,fs)
y = fft(v);

n = length(v);
delT = n/fs;
delf = 1/delT;

f = (0:ceil(n/2-1))*delf;

figure(2), semilogy(f,abs(y(1:n/2)./f')),grid on;