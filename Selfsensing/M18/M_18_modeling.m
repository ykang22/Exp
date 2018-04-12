%M18 Milwaukee drill
%Ye gu Kang
%20171118

clear all;close all;clc;




Num_slot=6
Num_pole=4
Num_turns=9



coil_length=2*(155+122)+54+155
length_tot=coil_length
Num_stranded_wire=1
coil_diameter=1.15
winding_temperature=25

resistivity_cu=(3.76*winding_temperature+873)*10^(-6)/55

Num_series=2
Num_parallel=1

coil_resistance=resistivity_cu*(length_tot)/(pi*coil_diameter^2/4*Num_stranded_wire)
%theoretical result
R_phase=coil_resistance*Num_series/Num_parallel



K_e=1/(200/60*2*pi)
K_t=K_e*3
K_t*10



openfemm
newdocument(0)
path='C:/femm42/';
name_fem='M_18.fem';

mi_saveas([path,name_fem])

Hc=888000*0.93;
Hc=902000;

idensity = 0;
u_r_mag=1.05;
u_r_epoxy=1;
mi_addmaterial('NXX',u_r_mag,u_r_mag,Hc,0,0.56)
mi_addmaterial('NoMag',u_r_epoxy,u_r_epoxy)
mi_getmaterial('Air')
mi_getmaterial('M-19 Steel')
mi_getmaterial('Pure Iron')
mi_addmaterial('a+', 1, 1, 0, -idensity/2, 56)
mi_addmaterial('a-', 1, 1, 0, idensity/2, 56)
mi_addmaterial('b+', 1, 1, 0, idensity, 56)
mi_addmaterial('b-', 1, 1, 0, -idensity, 56)
mi_addmaterial('c+', 1, 1, 0, -idensity/2, 56)
mi_addmaterial('c-', 1, 1, 0, idensity/2, 56)
mi_addboundprop('A_0',0,0,0,0,0,0,0,0,0)

%add parameter
depth = 24.5;
pole_number= 4;
slot_number = 6;
airgap = 0.5;
m_thickness=2.0;
m_width=15.0;
D_stator_outer =  50;
D_rotor_inner = 5;
D_rotor_outer=26;
D_shaft=D_rotor_inner-m_thickness*2;
th_core = 3.6;
w_teeth = 6.12;
sue_1=0.65;
sue_2=1.15;
R_sue_3=D_rotor_outer*0.5+airgap;
R_sue_2=R_sue_3+sue_2;
R_sue_1=R_sue_2+sue_1;
slot_ratio = 0.84;




mi_probdef(0,'millimeters','planar',1e-008,depth,30)

%core
mi_addnode(-(D_stator_outer*0.5-th_core)*cos((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))),(D_stator_outer*0.5-th_core)*sin((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))))
mi_addnode(0,R_sue_3)
mi_addnode(R_sue_3*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_3*sin((pi/2+pi/slot_number*slot_ratio)))
mi_addnode(R_sue_2*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_2*sin((pi/2+pi/slot_number*slot_ratio)))
mi_addnode(-R_sue_1*cos((pi/2-(asin(w_teeth*0.5/R_sue_1)))),R_sue_1*sin((pi/2-(asin(w_teeth*0.5/R_sue_1)))))
mi_addsegment(-(D_stator_outer*0.5-th_core)*cos((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))),(D_stator_outer*0.5-th_core)*sin((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))),-R_sue_1*cos((pi/2-(asin(w_teeth*0.5/R_sue_1)))),R_sue_1*sin((pi/2-(asin(w_teeth*0.5/R_sue_1)))))
mi_addsegment(-R_sue_1*cos((pi/2-(asin(w_teeth*0.5/R_sue_1)))),R_sue_1*sin((pi/2-(asin(w_teeth*0.5/R_sue_1)))),R_sue_2*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_2*sin((pi/2+pi/slot_number*slot_ratio)))
mi_addsegment(R_sue_2*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_2*sin((pi/2+pi/slot_number*slot_ratio)),R_sue_3*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_3*sin((pi/2+pi/slot_number*slot_ratio)))
mi_addarc(0,R_sue_3,R_sue_3*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_3*sin((pi/2+pi/slot_number*slot_ratio)),(180/slot_number*slot_ratio),5)
mi_addnode((D_stator_outer*0.5-th_core)*cos((pi/2+pi/slot_number)),(D_stator_outer*0.5-th_core)*sin((pi/2+pi/slot_number)))
mi_addnode(R_sue_2*cos((pi/2+pi/slot_number)),R_sue_2*sin((pi/2+pi/slot_number)))
mi_addsegment((D_stator_outer*0.5-th_core)*cos((pi/2+pi/slot_number)),(D_stator_outer*0.5-th_core)*sin((pi/2+pi/slot_number)),R_sue_2*cos((pi/2+pi/slot_number)),R_sue_2*sin((pi/2+pi/slot_number)))
mi_addsegment(R_sue_2*cos((pi/2+pi/slot_number)),R_sue_2*sin((pi/2+pi/slot_number)),R_sue_2*cos((pi/2+pi/slot_number*slot_ratio)),R_sue_2*sin((pi/2+pi/slot_number*slot_ratio)))
mi_addarc(-(D_stator_outer*0.5-th_core)*cos((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))),(D_stator_outer*0.5-th_core)*sin((pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))))),(D_stator_outer*0.5-th_core)*cos((pi/2+pi/slot_number)),(D_stator_outer*0.5-th_core)*sin((pi/2+pi/slot_number)),(180/slot_number-180/pi*(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core)))),5)
mi_selectcircle(0,0,D_stator_outer,4)
mi_mirror2(0,0,0,10,4)
mi_selectcircle(0,0,D_stator_outer*0.5,4)
mi_copyrotate2(0,0,360/slot_number,slot_number,4)


x1=0;
y1=0;
r=D_stator_outer*0.5;
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)
mi_clearselected()

mi_selectarcsegment(0,r);
mi_setarcsegmentprop(1,'A_0', 0,0);
mi_selectarcsegment(0,-r);
mi_setarcsegmentprop(1,'A_0', 0,0);

mi_clearselected()
mi_zoomnatural()

%winding layout
q_denominator=pole_number*3/gcd(slot_number,pole_number*3);
q_numerator=slot_number/gcd(slot_number,pole_number*3);
num_zeros=q_denominator-q_numerator;
seq_zero= num_zeros/q_numerator;
sequence=[];
num_repeat=slot_number/q_numerator;
% temp_num_zeros=ceil(seq_zero-0.5)
temp_num_zeros=round(seq_zero);
count_zeros=0;
count_ones=0;
for i = 1: num_repeat*q_numerator
    count_ones=count_ones+1;
    sequence(count_ones+count_zeros)=1;
    for x = 1:temp_num_zeros
        count_zeros=count_zeros+1;
        sequence(count_ones+count_zeros)=0;
    end
    
    if ((count_zeros/count_ones)>seq_zero)
        temp_num_zeros=floor(seq_zero);
    elseif((count_zeros/count_ones)<seq_zero)
        temp_num_zeros=ceil(seq_zero);
    elseif((count_zeros/count_ones)==seq_zero)
        temp_num_zeros=round(seq_zero);
    end
end

seq=[];
Dist=[-12, 11, -10, 12, -11, 10];
seq_count=1;
seq_temp=0;
for j=1:length(sequence)
    if (sequence(j)==1)
        seq(seq_count)=Dist(mod(j,6)+1);
        seq_count=seq_count+1;
        seq(seq_count)=seq(seq_count-1)*-1;
        seq_count=seq_count+1;
    end
end

%winding
zz=1;
for v=1:length(seq)
    xx=floor((v-1)/2);
    zz=zz*-1;
    mi_addblocklabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx));
    mi_selectlabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx));
    if seq(v)==-10
        mi_setblockprop('b-', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    elseif seq(v)==10
        mi_setblockprop('b+', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    elseif seq(v)==-11
        mi_setblockprop('a-', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    elseif seq(v)==11
        mi_setblockprop('a+', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    elseif seq(v)==-12
        mi_setblockprop('c-', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    elseif seq(v)==12
        mi_setblockprop('c+', 1, 0, '<None>', 0, 0, 0);
        mi_setgroup(6);
        mi_clearselected();
    end
end


%rotor
m_height=15.9*0.5;
flux_barrier_height=24.3*0.5;
flux_barrier_1=1.0;
flux_barrier_3=0.85;
m_upper=14.3;

mi_addnode(0,m_height+m_thickness)
mi_addnode(0,m_height)
mi_addnode(m_width*0.5,m_height)
mi_addnode(m_upper*0.5,m_height+m_thickness)
mi_addnode(m_width*0.5,m_height)
mi_addnode(m_upper*0.5,m_height)
mi_addsegment(m_upper*0.5,m_height,m_upper*0.5,m_height+m_thickness)
mi_addsegment(0,m_height+m_thickness,m_upper*0.5,m_height+m_thickness)
mi_addsegment(0,m_height,m_width*0.5,m_height)
mi_addnode(m_width*0.5,m_height+flux_barrier_3)
mi_addsegment(m_width*0.5,m_height+flux_barrier_3,m_width*0.5,m_height)
mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_moverotate(0,0,360/pole_number/2)

mi_addnode(-flux_barrier_1*0.5,flux_barrier_height)
mi_clearselected();
mi_addblocklabel(-flux_barrier_1*0.75,flux_barrier_height-0.25);
mi_selectlabel(-flux_barrier_1*0.75,flux_barrier_height-0.25);
mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);
mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_moverotate(0,0,-360/pole_number/2)
mi_clearselected()
mi_selectgroup(2)
mi_moverotate(0,0,-360/pole_number/2)

mi_addsegment(m_width*0.5,m_height+flux_barrier_3,m_width*0.5+1,m_height+flux_barrier_3 )
mi_addsegment(m_upper*0.5,m_height+m_thickness,m_width*0.5+1,m_height+flux_barrier_3 )

mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_mirror2(0,0,0,10,4)
mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_copyrotate2(0,0,360/pole_number,pole_number,4)

x1=0;
y1=0;
r=D_rotor_inner*0.5;
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

x1=0;
y1=0;
r=D_rotor_outer*0.5;
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

for z=1:pole_number/2
    mi_clearselected()
    mi_addblocklabel(cos((pi/2+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5),sin((pi/2+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5))
    mi_selectlabel(cos((pi/2+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5),sin((pi/2+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5))
    mi_setblockprop('NXX',0,1,'None',90+(z-1)*720/pole_number,12,1)
    mi_clearselected()
    mi_addblocklabel(cos((pi/2-2*pi/pole_number+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5),sin((pi/2-2*pi/pole_number+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5))
    mi_selectlabel(cos((pi/2-2*pi/pole_number+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5),sin((pi/2-2*pi/pole_number+(z-1)*4*pi/pole_number))*(m_height+m_thickness*0.5))
    mi_setblockprop('NXX',0,1,'None',-180+(z-1)*720/pole_number,12,1)
end

%Add label
mi_clearselected();
mi_addblocklabel(0,D_stator_outer*0.5-1);
mi_selectlabel(0,D_stator_outer*0.5-1);
mi_setblockprop('M-19 Steel',0, 1, '<None>', 0, 0, 0);

mi_clearselected();
mi_addblocklabel(0,D_rotor_outer*0.5-1);
mi_selectlabel(0,D_rotor_outer*0.5-1);
mi_setblockprop('M-19 Steel', 0, 1, 'None', 0, 12, 0);

mi_clearselected();
mi_addblocklabel(0,0);
mi_selectlabel(0,0);
mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);

mi_clearselected();
mi_addblocklabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_selectlabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);

mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_moverotate(0,0,90)


mi_clearselected()
mi_refreshview()


Slot_area=1.02982e-6/24.5e-3*10^6

ampere_per_density=Slot_area/Num_turns*Num_parallel
iq=[];
id=[];
current_max=50;

num_iter=21
for n=1:(num_iter)
    for m=1:(num_iter)
        id((n-1)*(num_iter)+m)=-current_max+current_max/(num_iter-1)*(m-1)*2;
        iq((n-1)*(num_iter)+m)=-current_max+current_max/(num_iter-1)*(n-1)*2;
    end
end

temp_J=[];
I =[];
elec_deg=[];
for n=1:length(id)
    I(n)=(iq(n)^2+id(n)^2)^0.5;
    temp_J(n)=I(n)/ampere_per_density;
    elec_deg(n)=atan2(id(n),iq(n));
end
rot_angle=0;
time_passed=0
flux_phase_a=[];
flux_phase_b=[];
flux_phase_c=[];


for r=1:length(I)
    tic
    J_a(r)=temp_J(r)*sin(elec_deg(r)+(rot_angle));
    J_b(r)=temp_J(r)*sin(elec_deg(r)-2*pi/3+(rot_angle));
    J_c(r)=temp_J(r)*sin(elec_deg(r)+2*pi/3+(rot_angle));
    
    mi_modifymaterial('b+',4,J_b(r))
    mi_modifymaterial('b-',4,-J_b(r))
    mi_modifymaterial('a+',4,J_a(r))
    mi_modifymaterial('a-',4,-J_a(r))
    mi_modifymaterial('c+',4,J_c(r))
    mi_modifymaterial('c-',4,-J_c(r))
    
    mi_analyze(1)
    mi_loadsolution()
    
    %     mo_showdensityplot(0,0,1.6,0,'bmag')
    %     path='C:/femm42/'
    %     name_fem=tostring(r)..".png"
    %     mo_savebitmap(path..name_fem)
    flux_a=0;
    flux_b=0;
    flux_c=0;
    for v=1:length(seq)*0.5        
        x1=(D_stator_outer*0.5-th_core*1.5)*cos((w_teeth/2)/(D_stator_outer*0.5-th_core*1.5)*1.1+(pi/2)+(2*pi/slot_number)*(v-1));
        y1=(D_stator_outer*0.5-th_core*1.5)*sin((w_teeth/2)/(D_stator_outer*0.5-th_core*1.5)*1.1+(pi/2)+(2*pi/slot_number)*(v-1));
        mo_addcontour(x1,y1)
        x2=(D_stator_outer*0.5-th_core*1.5)*cos(-(w_teeth/2)/(D_stator_outer*0.5-th_core*1.5)*1.1+(pi/2)+(2*pi/slot_number)*(v-1));
        y2=(D_stator_outer*0.5-th_core*1.5)*sin(-(w_teeth/2)/(D_stator_outer*0.5-th_core*1.5)*1.1+(pi/2)+(2*pi/slot_number)*(v-1));
        mo_addcontour(x2,y2)
        flux_temp=mo_lineintegral(0);
        flux(v)=flux_temp(1);
        if seq(v*2)==-10
            flux_b=flux_b-flux_temp(1)
            mo_clearcontour()
        elseif seq(v*2)==10
            flux_b=flux_b+flux_temp(1)
            mo_clearcontour()
        elseif seq(v*2)==-11
            flux_a=flux_a-flux_temp(1)
            mo_clearcontour()
        elseif seq(v*2)==11
            flux_a=flux_a+flux_temp(1)
            mo_clearcontour()
        elseif seq(v*2)==-12
            flux_c=flux_c-flux_temp(1)
            mo_clearcontour()
        elseif seq(v*2)==12
            flux_c=flux_c+flux_temp(1)
            mo_clearcontour()
        end        
        
    end
    %     flux__(:,r)=flux
    %     flux_b=-(flux(1)+flux(2)+flux(6)+flux(7)-flux(8)-flux(9)+flux(10)+flux(11)+flux(15)+flux(16)-flux(17)-flux(18));
    %     flux_a=-(flux(1)-flux(2)-flux(3)+flux(4)+flux(5)+flux(9)+flux(10)-flux(11)-flux(12)+flux(13)+flux(14)+flux(18));
    %     flux_c=-(flux(3)+flux(4)-flux(5)-flux(6)+flux(7)+flux(8)+flux(12)+flux(13)-flux(14)-flux(15)+flux(16)+flux(17));
    %
    
    
    flux_phase_a(r)=flux_a*Num_turns/Num_parallel*Num_series/(Num_slot/3);
    flux_phase_b(r)=flux_b*Num_turns/Num_parallel*Num_series/(Num_slot/3);
    flux_phase_c(r)=flux_c*Num_turns/Num_parallel*Num_series/(Num_slot/3);
    
    mo_groupselectblock(12)
    T(r) = mo_blockintegral(22)
    Fx(r) = mo_blockintegral(18);
    Fy(r) = mo_blockintegral(19);
    
    mo_close()
    mi_clearselected()
    %     mi_selectcircle(0,0,(D_rotor_outer*0.5+airgap*0.5),4)
    %     mi_moverotate(0,0,-4*180/pole_number/num_iter)
    %     mi_clearselected()
    %     mi_selectgroup(12)
    %     mi_moverotate(0,0,-4*180/pole_number/num_iter)
    %     rot_angle=-2*pi/num_iter+rot_angle
    one_iter=toc
    time_require=one_iter*length(I)
    time_passed=one_iter+time_passed
    time_left=one_iter*length(I)-time_passed
end


figure(113)
plot(T)
xlabel('elec. angle[deg]')
ylabel('Torque[Nm]')
grid on
ylim([-4 4])

%  save('temp.mat')

% %%
% Npoints=num_iter
% Hz=pole_number/2*1000/60
% elec_angle=0:360/num_iter:(360-360/num_iter)
%
% for n=1:length(flux_phase_a)-1
%     back_emf_a(n)=-(flux_phase_a(n+1)-flux_phase_a(n))*Hz*Npoints
%     back_emf_b(n)=-(flux_phase_b(n+1)-flux_phase_b(n))*Hz*Npoints
%     back_emf_c(n)=-(flux_phase_c(n+1)-flux_phase_c(n))*Hz*Npoints
%
% end
% % figure(446)
% % plot(elec_angle,back_emf_a)
% % grid on
% % hold on
% % plot(elec_angle,back_emf_b)
% % plot(elec_angle,back_emf_c)
%
% plot(back_emf_a)
% grid on
% hold on
% plot(back_emf_b)
% plot(back_emf_c)
% title('Back-EMF vs. elec. angle')
% xlabel('[deg]')
% ylabel('E[V]')
% legend('Phase_a','Phase_b','Phase_c')
% ylim([-5 5])
%
% %%
% Npoints=num_iter
% elec_angle=0:360/num_iter:(360-360/num_iter)
% figure(100)
% plot(Fx,Fy)
% axis square
% axis([-350 350 -350 350])
% ylabel('Fy[N]')
% xlabel('Fx[N]')
% grid on
%
% figure(111)
% plot(elec_angle,Fx)
% xlabel('elec. angle[deg]')
% ylabel('Fx[N]')
% grid on
% ylim([-350 350])
% figure(112)
% plot(elec_angle,Fy)
% xlabel('elec. angle[deg]')
% ylabel('Fy[N]')
% grid on
% ylim([-350 350])
%
% figure(113)
% plot(elec_angle,Tor)
% xlabel('elec. angle[deg]')
% ylabel('Torque[Nm]')
% grid on
% ylim([-4 4])



%%
close all
clear all
load temp.mat
theta=0
for n=1:length(J_a)    
    i1=J_a(n)*ampere_per_density;
    i2=J_b(n)*ampere_per_density;
    i3=J_c(n)*ampere_per_density;    
    idq = abc2dq(i1,i2,i3,theta*pi/180);    
    i_dd(n)=idq(1);
    i_qq(n)=idq(2);
    flux11=flux_phase_a(n);
    flux21=flux_phase_b(n);
    flux31=flux_phase_c(n);
    fluxdq1 = abc2dq(flux11,flux21,flux31,theta*pi/180);    
    fluxd(n)=fluxdq1(1);
    fluxq(n)=fluxdq1(2);    
end

num_iter=21
for p=1:1:num_iter
    for q=1:1:num_iter
        i_d(p,q)=i_dd((p-1)*num_iter+q);
        i_q(p,q)=i_qq((p-1)*num_iter+q);
        flux_q(p,q)=fluxq((p-1)*num_iter+q);
        flux_d(p,q)=fluxd((p-1)*num_iter+q);
        Torque(p,q)=T((p-1)*num_iter+q);
    end
end

for p=1:1:size(flux_d,1)
    L_d_d(p,:)=diff(flux_d(p,:))./diff(i_d(p,:))*1000;
    L_q_q(:,p)=diff(flux_q(:,p))./diff(i_q(:,p))*1000;
    L_d_q(p,:)=diff(flux_d(:,p))./diff(i_q(:,p))*1000;
    L_q_d(:,p)=diff(flux_q(p,:))./diff(i_d(p,:))*1000;
end

id_step=diff(i_d(1,1:2))
iq_step=diff(i_q(1:2,1))
[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = flux_d%-flux_d(11,11);
figure(111)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('\lambda_d[Vs]')
% ylim([-5,5])
% xlim([-5 5])
% zlim([-0.01 0.01])
grid on
% hold on
% flux_q=fliplr(flux_q);
[X,Y] = meshgrid(-10*iq_step:iq_step:10*iq_step,-10*id_step:id_step:10*id_step);
Z = flux_q;
figure(112)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('\lambda_q[Vs]')
% ylim([-5, 5])
% zlim([-0.01 0.01])
grid on

[X,Y] = meshgrid(-9.5*id_step:id_step:9.5*id_step,-10*iq_step:iq_step:10*iq_step);
Z = L_d_d;
figure(114)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_d_d[mH]')
% zlim([0 1])
grid on
% title('Variation of q-axis cross-coupling inductance L_q_d')

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = L_d_q;
figure(117)
s = surf(X,Y,Z);
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_d_q[mH]')
% zlim([0 1])
grid on

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = L_q_q;
figure(115)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_q_q[mH]')
%  zlim([0 1])
grid on

[X,Y] = meshgrid(-9.5*id_step:id_step:9.5*id_step,-10*iq_step:iq_step:10*iq_step);
Z = L_q_d;
figure(116)
s = surf(X,Y,Z');
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_q_d[mH]')
% zlim([0 1])
grid on

[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Torque);
figure(117)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque[Nm]')
% zlim([0 1])
grid on


[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Torque)./(X.^2+Y.^2).^0.5;

figure(118)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque/I[Nm/A]')
% zlim([0 1])
grid on

%%
% clear
L_d_anal1=L_d_d(11,11)
L_q_anal1=L_q_q(11,11)
psi_a=flux_d(11,11)
Num_pole=4
Num_phase=3
P_n=Num_pole/2

L_q_q(21,:)=L_q_q(20,:);
L_d_d(:,21)=L_d_d(:,20);
L_d_q(:,21)=L_d_q(:,20);
L_q_d(21,:)=L_q_d(20,:);
i_step=abs(iq_step)
% temp_n=1;
% num_iter=21;
% for i_q=-10:1:10
%     for i_d=-10:1:10
% %         i_d
%         L_d_anal2=L_d_d(i_q+11,i_d+11);
%         L_q_anal2=L_q_q(i_q+11,i_d+11);
%          Tor_(i_d+11,i_q+11)=Num_phase*P_n*(psi_a*i_q*i_step+(L_d-L_q)/1000*i_d*i_q*i_step^2);
%         Tor2_(i_d+11,i_q+11)=Num_phase*P_n*(psi_a*i_q*i_step+(L_dd2-L_qq2)/1000*i_d*i_q*i_step^2);
%    
% %         Tor_(temp_n)=Num_phase*P_n*(psi_a*i_q*i_step+(L_d_anal1-L_q_anal1)/1000*i_d*i_q*i_step^2);
% %         Tor2_(temp_n)=Num_phase*P_n*(psi_a*i_q*i_step+(L_d_anal2-L_q_anal2)/1000*i_d*i_q*i_step^2);
% %         
%         
% 
%         flux_d_(temp_n)=L_d_d(i_q+11,i_d+11)/1000*i_d*i_step;
%         flux_q_(temp_n)=L_q_q(i_q+11,i_d+11)/1000*i_q*i_step;    
%         
%         temp_n=temp_n+1;
%     end
% end

% num_iter=21;
for m=1:num_iter
    for n=1:num_iter
        Tor_(m,n)=3/2*P_n*(psi_a*i_q(m,n)+(L_d_d(11,11)-L_q_q(11,11))/1000*i_d(m,n)*i_q(m,n));
        Tor2_(m,n)=3/2*P_n*(psi_a*i_q(m,n)+(L_d_d(m,n)-L_q_q(m,n))/1000*i_d(m,n)*i_q(m,n));
        Tor3_(m,n)=3/2*P_n*(flux_d(m,n)*i_q(m,n)-flux_q(m,n)*i_d(m,n));

        flux_d_(m,n)=L_d_d(m,n)/1000*i_d(m,n);
        flux_q_(m,n)=L_q_q(m,n)/1000*i_q(m,n);    
        flux_d_(m,n)=L_d_d(m,n)/1000*i_d(m,n)+L_d_q(n,m)/1000*i_q(m,n);
        flux_q_(m,n)=L_q_q(m,n)/1000*i_q(m,n)+L_q_d(n,m)/1000*i_d(n,m); 
    
    end
end

% Tor_= fliplr(Tor_)
% Tor2_= fliplr(Tor2_)
% Tor3_= fliplr(Tor3_)
figure(555)
plot(Tor_)
figure(556)
plot(Tor2_)

% num_iter=21
% for p=1:1:num_iter
%     for q=1:1:num_iter
% %         i_d(p,q)=i_dd((p-1)*num_iter+q);
% %         i_q(p,q)=i_qq((p-1)*num_iter+q);
%         flux_q_anal(p,q)=flux_q_((p-1)*num_iter+q);
%         flux_d_anal(p,q)=flux_d_((p-1)*num_iter+q);
%         Tor_anal(p,q)=Tor_((p-1)*num_iter+q);
%         Tor_anal2(p,q)=Tor2_((p-1)*num_iter+q);
%     end
% end
[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor_);
figure(120)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque[Nm]')
%  zlim([0 1])
grid on

[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor_)./(X.^2+Y.^2).^0.5;

figure(121)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque/I[Nm/A]')
% zlim([0 1])
grid on


[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor2_);
figure(130)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque[Nm]')
%  zlim([0 1])
grid on

[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor2_)./(X.^2+Y.^2).^0.5;

figure(131)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque/I[Nm/A]')
% zlim([0 1])
grid on


[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor3_);
figure(140)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque[Nm]')
%  zlim([0 1])
grid on

[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = abs(Tor3_)./(X.^2+Y.^2).^0.5;

figure(141)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('Torque/I[Nm/A]')
% zlim([0 1])
grid on


[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = flux_d_+flux_d(11,11);
figure(211)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('\lambda_d[Vs]')
% ylim([-5,5])
% xlim([-5 5])
% zlim([-0.01 0.01])
grid on
% hold on

[X,Y] = meshgrid(-10*id_step:id_step:10*id_step,-10*iq_step:iq_step:10*iq_step);
Z = flux_q_;
figure(212)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('\lambda_q[Vs]')
% ylim([-5, 5])
% zlim([-0.01 0.01])
grid on




%%
% clear all
syms T P_n I_a psi_a Beta L_q L_d i_d i_q
I_a=(i_d^2+i_q^2)^0.5
T=P_n*(psi_a*I_a*cos(Beta)+1/2*(L_q_anal1-L_d_anal1)*I_a^2*sin(2*Beta))
diff_T=diff(T, Beta)
Beta_solve=simplify(solve(diff_T,Beta))
i_dd=-(i_d^2+i_q^2)^0.5*sin(Beta_solve)
i_d_final=(simplify(solve(i_dd-i_d==0,i_d)))
