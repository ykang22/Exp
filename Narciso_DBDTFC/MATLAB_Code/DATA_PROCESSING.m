%% CURRENT REGULATOR DATA PROCESSING

%% NEGATIVE FREQUENCIES
load('current_reg_data_3.mat');
iqe_star = data1(2:end,3);
ide_star = data1(2:end,4);

iqe      = data1(2:end,1);
ide      = data1(2:end,2);

clear data1

load('current_reg_data_4.mat');
iqe_star = [iqe_star; data1(2:end,3)];
ide_star = [ide_star; data1(2:end,4)];
iqd_star = iqe_star - 1i*ide_star;

iqe      = [iqe;      data1(2:end,1)];
ide      = [ide;      data1(2:end,2)];
iqd      = iqe - 1i*ide;

[Hn,fhn] = mscohere(iqd_star,iqd,rectwin(1000),50,2^16,20000,'centered');
[Tn,fn] = tfestimate(iqd_star,iqd,rectwin(1000),50,2^16,20000,'centered');

clear iqd_star iqd data1
n = length(fn);

%% POSITIVE FREQUENCIES
load('current_reg_data_1.mat');
iqe_star = data1(2:end,3);
ide_star = data1(2:end,4);

iqe      = data1(2:end,1);
ide      = data1(2:end,2);

clear data1

load('current_reg_data_2.mat');
iqe_star = [iqe_star; data1(2:end,3)];
ide_star = [ide_star; data1(2:end,4)];
iqd_star = iqe_star - 1i*ide_star;

iqe      = [iqe;      data1(2:end,1)];
ide      = [ide;      data1(2:end,2)];
iqd      = iqe - 1i*ide;

[H,fh] = mscohere(iqd_star,iqd,rectwin(1000),50,2^16,20000,'centered');
[T,f] = tfestimate(iqd_star,iqd,rectwin(1000),50,2^16,20000,'centered');


% k = 0;
% while k <= n
%      k = k + 1;
%     if H(k) < 0.99
%         H(k) = [];
%         f(k) = [];
%         fh(k) = [];
%         T(k) = [];
%         n = length(f);s
%         k = k-1;
%         if k == n
%             break
%         end
%     end
% end

%% NEGATIVE FREQUENCIES
% figure(1), semilogx(-fhn(1:n/2), abs(Hn(1:n/2))), hold on;
% figure(2), semilogx(-fn(1:n/2), abs(Tn(1:n/2))), hold on;
% figure(3), semilogx(-fn(1:n/2), 180/pi*angle(Tn(1:n/2))), hold on;
% 
% %% POSITIVE FREQUENCIES
% figure(1), semilogx(fh, abs(H));
% figure(2), semilogx(f, abs(T));
% figure(3), semilogx(f, 180/pi*angle(T));

flog = [fn(1:n/2); f(n/2:end)];
T = [Tn(1:n/2); T(n/2:end)];
H = [Hn(1:n/2); H(n/2:end)];
flog = sign(flog).*log10(abs(flog));
figure
plot(flog,abs(T),'.')
xlim([-3.5 3.5]), ylim([0 1.1]);
% Do nothing else to get just exponents.  Otherwise:
% ft = get(gca,'XTick')';
% set(gca,'XTickLabel',num2str(sign(ft).*10.^ft))
% Or, for scientific notation
% STRING = strcat(num2str(sign(ft)),'0^',...
%     num2str(ft));
% set(gca,'XTickLabel',STRING);
