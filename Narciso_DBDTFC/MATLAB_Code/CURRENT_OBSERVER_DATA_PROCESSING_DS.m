%% CURRENT OBSERVER DATA PROCESSING DS

%% NEGATIVE FREQUENCIES
% load('curr_obs_dynstff_neg_1.mat');
iqe_hat = detrend(data1(2:end,1));
ide_hat = detrend(data1(2:end,2));
iqe      = detrend(data1(2:end,9));
ide      = detrend(data1(2:end,10));

% clear data1
% 
% load('curr_obs_dynstff_neg_2.mat');
% iqe_hat = [iqe_hat; detrend(data1(2:end,3))];
% ide_hat = [ide_hat; detrend(data1(2:end,4))];
% iqe      = [iqe;      detrend(data1(2:end,5))];
% ide      = [ide;      detrend(data1(2:end,6))];
% 
% % clear data1
% % 
% % load('curr_obs_dynstff_pos_3.mat');
% % iqe_hat = [iqe_hat; detrend(data1(2:end,3))];
% % ide_hat = [ide_hat; detrend(data1(2:end,4))];
% % iqe      = [iqe;      detrend(data1(2:end,5))];
% % ide      = [ide;      detrend(data1(2:end,6))];

iqd_hat = iqe_hat - 1i*ide_hat;
iqd     = iqe - 1i*ide;

[Hn,fhn] = mscohere(iqd,iqd_hat,rectwin(1000),50,2^16,20000,'centered');
[Tn,fn] = tfestimate(iqd,iqd_hat,rectwin(1000),50,2^16,20000,'centered');

% clear iqd_star iqd data1
n = length(fn);

%% POSITIVE FREQUENCIES
% % load('curr_obs_dynstff_pos_1.mat');
% iqe_hat = detrend(data1(2:end,3));
% ide_hat = detrend(data1(2:end,4));
% iqe      = detrend(data1(2:end,5));
% ide      = detrend(data1(2:end,6));
% 
% % clear data1
% % 
% % load('curr_obs_dynstff_pos_2.mat');
% % iqe_hat = [iqe_hat; detrend(data1(2:end,3))];
% % ide_hat = [ide_hat; detrend(data1(2:end,4))];
% % iqe      = [iqe;      detrend(data1(2:end,5))];
% % ide      = [ide;      detrend(data1(2:end,6))];
% % 
% % clear data1
% % 
% % load('curr_obs_dynstff_pos_3.mat');
% % iqe_hat = [iqe_hat; detrend(data1(2:end,3))];
% % ide_hat = [ide_hat; detrend(data1(2:end,4))];
% % iqe      = [iqe;      detrend(data1(2:end,5))];
% % ide      = [ide;      detrend(data1(2:end,6))];
% 
% iqd_hat = iqe_hat - 1i*ide_hat;
% iqd      = iqe - 1i*ide;
% 
% [H,fh] = mscohere(iqd_hat,iqd,rectwin(1000),50,2^16,20000,'centered');
% [T,f] = tfestimate(iqd_hat,iqd,rectwin(1000),50,2^16,20000,'centered');
% 
% % n = length(f);
% % 
% % k = 0;
% % while k <= n
% %      k = k + 1;
% %     if H(k) < 0.99
% %         H(k) = [];
% %         f(k) = [];
% %         fh(k) = [];
% %         T(k) = [];
% %         n = length(f);
% %         k = k-1;
% %         if k == n
% %             break
% %         end
% %     end
% % end

%% NEGATIVE FREQUENCIES
% figure(1), semilogx(-fhn(1:n/2), abs(Hn(1:n/2))), hold on, grid on;
% figure(2), loglog(-fn(1:n/2), abs(Tn(1:n/2)),...
%     'LineWidth',1.5), hold on, grid on,...% ylim([0.1 10]),...
%     xlim([min(-fn(1:n/2)) max(-fn(1:n/2))]);
% figure(3), semilogx(-fn(1:n/2), 180/pi*angle(Tn(1:n/2))), hold on, grid on;

%% POSITIVE FREQUENCIES
figure(1), semilogx(fhn, abs(Hn),'LineWidth',1.5), ylim([0 1]),...
    xlim([min(fn(length(fn)/2:end)) max(fn)]), grid on, hold on;
figure(2), loglog(fn, abs(Tn),...
    'LineWidth',1.5), grid on, hold on, ...
%     ylim([0 1.5]),...
    xlim([min(fn(length(fn)/2:end)) max(fn)]);
figure(3), semilogx(fn, 180/pi*angle(Tn),'LineWidth',1.5),...
    xlim([min(fn(length(fn)/2:end)) max(fn)]), grid on, hold on;

% flog = [fn(1:n/2); f(n/2:end)];
% T = [Tn(1:n/2); T(n/2:end)];
% H = [Hn(1:n/2); H(n/2:end)];
% flog = sign(flog).*log10(abs(flog));
% figure
% plot(flog,abs(T),'.')
% xlim([-3.5 3.5]), ylim([0 1.1]);
% % Do nothing else to get just exponents.  Otherwise:
% % ft = get(gca,'XTick')';
% % set(gca,'XTickLabel',num2str(sign(ft).*10.^ft))
% % Or, for scientific notation
% % STRING = strcat(num2str(sign(ft)),'0^',...
% %     num2str(ft));
% % set(gca,'XTickLabel',STRING);

% spectrogram(iqd_hat,kaiser(1000,5),750,2^8,20000,'centered')
