%Ye gu Kang
%calculate winding factor
clear all
close all
clc;

phase_num=3;
Num_layers = 2;

% pole_number=1:1:10;
% slot_number=1:1:12;


pole_number=1:1:55;
slot_number=1:1:55;
harmonic_number=1:1:11;

for x = 1:length(slot_number)
    for y = 1:length(pole_number)
        for h=1:length(harmonic_number)
            pole_angle(x,y,h)=1./pole_number(y);
            pitch_angle(x,y,h)=1./slot_number(x);
            Beta(x,y,h)=(pitch_angle(x,y,h)-pole_angle(x,y,h))./pitch_angle(x,y,h);
            pitch_factor(x,y,h)=sin(pitch_angle(x,y,h)./pole_angle(x,y,h).*h*pi/2)*sin(h*pi/2);
            w=x/gcd(x,y*3);
            distribution_factor(x,y,h)=sin(h.*pi/2/phase_num)./w./sin(h.*pi/2./phase_num/w);
            %             Beta(x,y)=(pole_angle(x,y)-pitch_angle(x,y))./pole_angle(x,y);
            %             pitch_factor(x,y,h)=cos(Beta(x,y,h).*h*pi);
            %             if rem(h,2)~=1
            %                 pitch_factor(x,y,h)=0;
            %             end
            %             q=x/phase_num/y;
            %             gamma=pi/(y/x);
            %             distribution_factor(x,y,h)=sin(h.*q*gamma/2)./(q*sin(h.*gamma/2));
            
            %         skew_factor(x,y)=sin(z*pi.*pole_number/2./slot_number)/(z*pi.*pole_number/2./slot_number);
            %pg108
            %     q = Num_slots/(Num_poles*Num_phases)
            %     gamma = 2*pi/Num_slots;
            %     k_ph(h) = abs(sin(h*(Num_slots/2 - 2)/(Num_slots/2)*pi/2));
            %     k_dh(h) = abs(sin(h*q*gamma/2)/(q*sin(h*gamma/2)));
            
            winding_factor(x,y,h)=pitch_factor(x,y,h).*distribution_factor(x,y,h);
            
            if rem(x,3)~=0
                pitch_factor(x,y,h)=0;           distribution_factor(x,y,h)=0;           winding_factor(x,y,h)=0;
            end
            if rem(y,2)~=0
                pitch_factor(x,y,h)=0;           distribution_factor(x,y,h)=0;           winding_factor(x,y,h)=0;
            end
        end
    end
end

pole_number=46
slot_number=51
disp('distribution factor')
distribution_factor(slot_number,pole_number,:)
disp('pitch factor')
pitch_factor(slot_number,pole_number,:)
disp('widning factor')
winding_factor(slot_number,pole_number,:)
%%
%winding layout
q_denominator=pole_number*3/gcd(slot_number,pole_number*3)
q_numerator=slot_number/gcd(slot_number,pole_number*3)
num_zeros=q_denominator-q_numerator
seq_zero= num_zeros/q_numerator
sequence=[]
num_repeat=slot_number/q_numerator
% temp_num_zeros=ceil(seq_zero-0.5)
temp_num_zeros=round(seq_zero)
count_zeros=0
count_ones=0
for i = 1: num_repeat*q_numerator
    count_ones=count_ones+1
    sequence(count_ones+count_zeros)=1
    for x = 1:temp_num_zeros
        count_zeros=count_zeros+1
        sequence(count_ones+count_zeros)=0
    end
    
    if ((count_zeros/count_ones)>seq_zero)
        temp_num_zeros=floor(seq_zero)
    elseif((count_zeros/count_ones)<seq_zero)
        temp_num_zeros=ceil(seq_zero)
    elseif((count_zeros/count_ones)==seq_zero)
        temp_num_zeros=round(seq_zero)
    end
end

seq=[]
Dist=[-12, 11, -10, 12, -11, 10]
seq_count=1
seq_temp=0
for j=1:length(sequence)
    if (sequence(j)==1)
        seq(seq_count)=Dist(mod(j,6)+1)
        seq_count=seq_count+1
        seq(seq_count)=seq(seq_count-1)*-1
        seq_count=seq_count+1
    end
end

S1_u=[]
n=1
for x=1:length(seq)
    if (seq(x)==11)
        S1_u(n)=floor(x/2)+1
        n=1+n
    end
    if (seq(x)==-11)
        S1_u(n)=-floor(x/2)-1
        n=1+n
    end
end
S1_v=[]
n=1
for x=1:length(seq)
    if (seq(x)==12)
        S1_v(n)=floor(x/2)+1
        n=1+n
    end
    if (seq(x)==-12)
        S1_v(n)=-floor(x/2)-1
        n=1+n
    end
end
S1_w=[]
n=1
for x=1:length(seq)
    if (seq(x)==10)
        S1_w(n)=floor(x/2)+1
        n=1+n
    end
    if (seq(x)==-10)
        S1_w(n)=-floor(x/2)-1
        n=1+n
    end
end
% harmonic_num=9
num_harmonics=3
number_fig=1
for phase_shift=0:2*pi/number_fig:2*pi/number_fig*3
    %     phase_shift=0
    E_u=[];
    E_v=[];
    E_w=[];
    for x=1:length(S1_u)
        for y=1:num_harmonics
            E_u(x,y)=exp(1i*y*(abs(S1_u(x))*pi*pole_number/(slot_number)+phase_shift))*S1_u(x)/abs(S1_u(x))/2*3/slot_number  ;
            E_v(x,y)=exp(1i*y*(abs(S1_v(x))*pi*pole_number/(slot_number)+phase_shift))*S1_v(x)/abs(S1_v(x))/2*3/slot_number  ;
            E_w(x,y)=exp(1i*y*(abs(S1_w(x))*pi*pole_number/(slot_number)+phase_shift))*S1_w(x)/abs(S1_w(x))/2*3/slot_number  ;
            
            %                E_u(x,y)=1/y.*exp(1i*y*(abs(S1_u(x))*pi*pole_number/(slot_number)+phase_shift))*S1_u(x)/abs(S1_u(x))/2*3/slot_number  ;
            %             E_v(x,y)=1/y.*exp(1i*y*(abs(S1_v(x))*pi*pole_number/(slot_number)+phase_shift))*S1_v(x)/abs(S1_v(x))/2*3/slot_number  ;
            %             E_w(x,y)=1/y
            
            %
            %             E_u(x,y)=exp(1i*y*((S1_u(x))*pi*pole_number/(slot_number)+phase_shift))*S1_u(x)/abs(S1_u(x))/2*3/slot_number  ;
            %             E_v(x,y)=exp(1i*y*((S1_v(x))*pi*pole_number/(slot_number)+phase_shift))*S1_v(x)/abs(S1_v(x))/2*3/slot_number  ;
            %             E_w(x,y)=exp(1i*y*((S1_w(x))*pi*pole_number/(slot_number)+phase_shift))*S1_w(x)/abs(S1_w(x))/2*3/slot_number  ;
        end
    end
    
    for y=1:num_harmonics
        E_u_real_old(y)=0;
        E_u_imag_old(y)=0;
        E_v_real_old(y)=0;
        E_v_imag_old(y)=0;
        E_w_real_old(y)=0;
        E_w_imag_old(y)=0;
        for x=1:length(S1_u)
            
            figure(y)
            hold on
            grid on
            axis square
            %             axis([-1/y 1/y -1/y 1/y])
            quiver(E_u_real_old(y),E_u_imag_old(y),real(E_u(x,y)),imag(E_u(x,y)),'AutoScale', 'on','Color','r','lineWidth',1)%,'MaxHeadSize',4)
            E_u_real_old(y)=E_u_real_old(y)+real(E_u(x,y));
            E_u_imag_old(y)=E_u_imag_old(y)+imag(E_u(x,y));
            
            quiver(E_v_real_old(y),E_v_imag_old(y),real(E_v(x,y)),imag(E_v(x,y)),'AutoScale', 'on','Color','b','lineWidth',1)%,'MaxHeadSize',4)
            E_v_real_old(y)=E_v_real_old(y)+real(E_v(x,y));
            E_v_imag_old(y)=E_v_imag_old(y)+imag(E_v(x,y));
            
            quiver(E_w_real_old(y),E_w_imag_old(y),real(E_w(x,y)),imag(E_w(x,y)),'AutoScale', 'on','Color','g','lineWidth',1)%,'MaxHeadSize',4)
            E_w_real_old(y)=E_w_real_old(y)+real(E_w(x,y));
            E_w_imag_old(y)=E_w_imag_old(y)+imag(E_w(x,y));
            
            
        end
    end
    
    WF=abs(sum(E_v))
    WF=abs(sum(E_u))
    WF=abs(sum(E_w))
end
%%
Num_coilset=slot_number/phase_num*Num_layers/2;
Num_coilturns = 1;
Phase_current=1;
Num_series=1;
Num_parallel=1;
angle_slot=360/slot_number;
%slots per pole per phase
q = slot_number/(pole_number*phase_num)
%slot span angle
Num_step=2^10;
theta = linspace(0,360,Num_step-1);

seq1=seq([end 1:end-1])

coil_a=[]
n=1
for m=1:2:length(seq1)
    temp_coil=0
    for x=1:2
        if (seq1(m+x-1)==11)
            temp_coil=temp_coil+1;
            
        elseif (seq1(m+x-1)==-11)
            temp_coil=-1+temp_coil;
        else
            temp_coil=0+temp_coil;
        end
        
    end
    coil_a(n)=temp_coil
    n=1+n
end

coil_b=coil_a([ end-(slot_number/3)+1:end 1:end-(slot_number/3) ])
coil_c=coil_a([ end-(2*slot_number/3)+1:end 1:end-(2*slot_number/3) ])

amplitude_a=0;amplitude_b=0;amplitude_c=0;
area_a=0;area_b=0;area_c=0;
temp_angle=360/slot_number;
j=1;
% Phase_a_windingfunction=zeros(1,length(theta));
for i = 1:1:length(theta)
    if(theta(i)>temp_angle)
        amplitude_a=amplitude_a-coil_a(j);
        amplitude_b=amplitude_b-coil_b(j);
        amplitude_c=amplitude_c-coil_c(j);
        temp_angle=temp_angle+360/slot_number;
        j=j+1;
    end
    Phase_a_windingfunction(i) = amplitude_a*Num_coilturns;
    Phase_b_windingfunction(i) = amplitude_b*Num_coilturns;
    Phase_c_windingfunction(i) = amplitude_c*Num_coilturns;
    area_a=Phase_a_windingfunction(i)+area_a;
    area_b=Phase_b_windingfunction(i)+area_b;
    area_c=Phase_c_windingfunction(i)+area_c;
end
Phase_a_windingfunction(end)=Phase_a_windingfunction(1);
Phase_b_windingfunction(end)=Phase_b_windingfunction(1);
Phase_c_windingfunction(end)=Phase_c_windingfunction(1);

offset_a=area_a/length(theta);
offset_b=area_b/length(theta);
offset_c=area_c/length(theta);
Phase_a_windingfunction=Phase_a_windingfunction-offset_a;
Phase_b_windingfunction=Phase_b_windingfunction-offset_b;
Phase_c_windingfunction=Phase_c_windingfunction-offset_c;

MMF_a=Phase_a_windingfunction*Phase_current*cos(pi/180);
MMF_b=Phase_b_windingfunction*Phase_current*cos((120)*pi/180);
MMF_c=Phase_c_windingfunction*Phase_current*cos((240)*pi/180);


deg_shift=37.5;
k=round(Num_step/360*deg_shift);
MMF_a=MMF_a([ end-k+1:end 1:end-k ]);
MMF_b=MMF_b([ end-k+1:end 1:end-k ]);
MMF_c=MMF_c([ end-k+1:end 1:end-k ]);

MMF=MMF_a+MMF_b+MMF_c;
figure(111)
subplot(3,1,1)
plot(theta,MMF_a,'r','linewidth',2)
grid on
legend('Phase_a')
subplot(3,1,2)
plot(theta,MMF_b,'black-.','linewidth',2)
grid on
legend('Phase_b')
subplot(3,1,3)
plot(theta,MMF_c,'b--','linewidth',2)
legend('Phase_c')
grid on
xlim([0 360])
% ylim([-44 44])
xlabel('electrical angle[deg]')
ylabel('MMF of Phase_a[A]')
set(gca,'XTick',0:30:360)

figure(112)
hold on
plot(theta,MMF)
grid on
xlim([0 360])
xlabel('electrical angle[deg]')
ylabel('MMF of three phase winding[A]')
set(gca,'XTick',0:30:360)


Num_harmonics=15;
N=length(Phase_a_windingfunction); %get the number of points
k=0:N-1;     %create a vector from 0 to N-1
T=1;      %get the frequency interval
freq=k/T;    %create the frequency range
amp_square=4/pi*Num_coilturns*(slot_number/phase_num/2);
X_mmf_a=abs(fft(MMF_a))/N*2/amp_square;
X_mmf=abs(fft(MMF))/N*2/(amp_square*3/2);
cutOff = ceil(N/2);
X_mmf_a = X_mmf_a(1:Num_harmonics+1);
X_mmf = X_mmf(1:Num_harmonics+1);
freq = freq(1:Num_harmonics+1);
figure(113)
stem(freq,X_mmf_a.*freq);
xlabel('Harmnonics')
ylabel('Amplitude')
figure(114)
stem(freq,X_mmf.*freq);
xlabel('Harmnonics')
ylabel('Amplitude')

THD_MMF_a = sqrt(sum(X_mmf_a(3:Num_harmonics+1).^2))/X_mmf_a(2)
THD_MMF = sqrt(sum(X_mmf(3:Num_harmonics+1).^2))/X_mmf(2)


gamma = 2*pi/slot_number;
N_t=Num_coilturns*(slot_number/phase_num);
%Calculate winding factor and THD analytically
for h = 1:2:Num_harmonics
    k_ph(h) = abs(sin(h*(slot_number/2 - 2)/(slot_number/2)*pi/2));
    k_dh(h) = abs(sin(h*q*gamma/2)/(q*sin(h*gamma/2)));
    k_h(h) = k_ph(h)*k_dh(h);
    MMF_harmonics(h) = (4/pi)*(N_t/(pole_number))*1/h*k_h(h);
end
THD_MMF_a_analytical = sqrt(sum(MMF_harmonics(2:Num_harmonics).^2))/MMF_harmonics(1)
%for MMF of three phase, exclude 3rd and mupliples of 3rd harmonics
sum_har=0;
for i=2:Num_harmonics
    if(mod(i,3)~=0)
        sum_har=MMF_harmonics(i)^2+sum_har;
    end
end
THD_MMF_analytical = sqrt(sum_har)/MMF_harmonics(1)


