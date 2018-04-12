clear all
close all
clc


R_s=0.377 %ohm
L_s=0.2349 %mH
K_e_krpm=2.813 %Vpk/kRPM
K_e=K_e_krpm/1000*60/(2*pi)
K_t=K_e*3

I_rated=0.27/K_t

I_rated_2=(0.00042^2*pi/4*2)*(5*10^6)/0.4

Num_slot=18
Num_series=3
Num_parallel=2
Num_turns=25
%awg 26
% with insulation
coil_diameter_w_insu=0.42;
%without insulation
coil_diameter=coil_diameter_w_insu*0.924;
%coil
coil_depth=19;
coil_width=14;
coil_f_radius=0.5;

% Nxt=floor((38.6-25.4)/coil_diameter_w_insu);
% Nzt=ceil(Num_turns/Nxt);

winding_temperature=25;
resistivity_cu=(3.76*winding_temperature+873)*10^(-6)/55;
% Nxt(1)=5;
% Nxt(2)=5;
%winding
% for n=1:1:2
% length_turn=2*(coil_depth+coil_width)*(coil_f_radius+coil_diameter_w_insu/2+3^0.5/2*coil_diameter_w_insu*(n-1));
% length_coil(n)=length_turn*(Nxt(n)-(1+(-1)^n)/2);
% end
length_coil=(coil_depth+coil_width)*2*Num_turns
length_tot=sum(length_coil);
coil_resistance=resistivity_cu*(length_tot)/(pi*coil_diameter^2/4)
%theoretical result
R_phase=coil_resistance*Num_series/Num_parallel


openfemm
newdocument(0)
path='C:/femm42/'
name_fem='ME577.fem'

mi_saveas([path,name_fem])

Hc=888000*0.93

idensity = 0
u_r_mag=1.05
u_r_epoxy=1
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
% mi_addmaterial('I2_u+', 1, 1, 0, -idensity/2, 56)
% mi_addmaterial('I2_u-', 1, 1, 0, idensity/2, 56)
% mi_addmaterial('I2_v+', 1, 1, 0, idensity, 56)
% mi_addmaterial('I2_v-', 1, 1, 0, -idensity, 56)
% mi_addmaterial('I2_w+', 1, 1, 0, -idensity/2, 56)
% mi_addmaterial('I2_w-', 1, 1, 0, idensity/2, 56)
mi_addboundprop('A_0',0,0,0,0,0,0,0,0,0)

% coil diameter
d=0.5
l_0=0.3
% stack length
b=19*0.95
% teeth height
h=12
% teeth width
c=2.93
% coil teeth width
w=32.5

% add parameter
depth = b
pole_number= 8
slot_number = 18
airgap = 0.1
m_thickness=4
pole_arc_ratio = 0.75
D_stator_outer =  53.4
D_rotor_outer=29
D_rotor_inner=D_rotor_outer-m_thickness*2
th_core = 2.54
w_teeth = c
sue_1=1
sue_2=0.2
R_sue_3=D_rotor_outer*0.5+airgap
R_sue_2=R_sue_3+sue_2
R_sue_1=R_sue_2+sue_1
slot_ratio = 0.57

mi_probdef(0,'millimeters','planar',1e-008,depth,30)

square=D_stator_outer*0.5*2.2
mi_drawrectangle(-square/2,-square/2,square/2,square/2)

x1=-square/2
y1=-square/2
x2=square/2
y2=square/2
mi_selectsegment((x1+x2)/2,y1)
mi_selectsegment((x1+x2)/2,y2)
mi_selectsegment(x1,(y1+y2)/2)
mi_selectsegment(x2,(y1+y2)/2)

mi_setsegmentprop('A_0', 0, 1,0,0)
mi_clearselected()
mi_zoomnatural()

% core
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

x1=0
y1=0
r=D_stator_outer*0.5
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

% winding
zz=1
mi_clearselected()


seq=[10 -11 -11 -10 12 11 11 -12 -12 -11 10 12 12 -10 -10 -12 11 10 10 -11 -11 -10 12 11 11 -12 -12 -11 10 12 12 -10 -10 -12 11 10]
%winding
zz=1
for v=1:length(seq)
  
    xx=floor((v-1)/2)
    zz=zz*-1
    mi_addblocklabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx))
    mi_selectlabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx))
    if seq(v)==-10
        mi_setblockprop('b-', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==10
        mi_setblockprop('b+', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==-11
        mi_setblockprop('a-', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==11
        mi_setblockprop('a+', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==-12
        mi_setblockprop('c-', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==12
        mi_setblockprop('c+', 1, 0, '<None>', 0, 0, 0)
        mi_setgroup(6)
        mi_clearselected()
    end
end

h_r_yoke=22.4*0.5
theta=pi*(8-2)/8
y=2*h_r_yoke*cot(0.5*theta)

y_m=8.15*0.5
h_m=1.23
mi_drawpolyline([h_r_yoke,-y/2;h_r_yoke,y/2])%;x*cos(theta),y/2*sin(0.5*theta);x*cos(theta),y/2*sin(theta)])

mi_addnode(h_r_yoke,y_m)
mi_addnode(h_r_yoke,-y_m)
mi_addnode(h_r_yoke+h_m,y_m)
mi_addnode(h_r_yoke+h_m,-y_m)
mi_addsegment(h_r_yoke,y_m,h_r_yoke+h_m,y_m)
mi_addsegment(h_r_yoke,-y_m,h_r_yoke+h_m,-y_m)
mi_addarc(h_r_yoke+h_m,-y_m,h_r_yoke+h_m,y_m,550/pole_number,5)

mi_selectcircle(0,0,D_rotor_outer*0.5,4)
mi_copyrotate2(0,0,360/pole_number,pole_number,4)



%magnet
% mi_addnode(D_rotor_inner/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_inner/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
% mi_addnode(D_rotor_outer/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_outer/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
% mi_addsegment(D_rotor_inner/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_inner/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_outer/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_outer/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
%
% mi_addnode(-D_rotor_inner/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_inner/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
% mi_addnode(-D_rotor_outer/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_outer/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
% mi_addsegment(-D_rotor_inner/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_inner/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),-D_rotor_outer/2*cos((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)),D_rotor_outer/2*sin((pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5)))
%
% mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
% mi_copyrotate2(0,0,360/pole_number,pole_number,4)

for z=1:(pole_number/2)
    mi_clearselected()
    mi_addblocklabel(cos((pi/2+pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2+pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
    mi_selectlabel(cos((pi/2+pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2+pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
    mi_setblockprop('NXX',0,1,'None',90+180/pole_number+(z-1)*4*180/pole_number,12,0)
    mi_clearselected()
    mi_addblocklabel(cos((pi/2-pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2-pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
    mi_selectlabel(cos((pi/2-pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2-pi/pole_number+(z-1)*4*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
    mi_setblockprop('NXX',0,1,'None',-90-180/pole_number+(z-1)*4*180/pole_number,12,0)
end

   mi_clearselected()
    mi_selectgroup(12)
    mi_moverotate(0,0,-22.5)
% mo_close()
% mi_clearselected()
% mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
% mi_moverotate(0,0,180/8)



% for z=1:pole_number
%     mi_clearselected()
%     mi_addblocklabel(cos((pi/2+(z-1)*2*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2+(z-1)*2*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
%     mi_selectlabel(cos((pi/2+(z-1)*2*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5,sin((pi/2+(z-1)*2*pi/pole_number))*(D_rotor_inner+m_thickness)*0.5)
%     mi_setblockprop('NoMag',0,1,'None',0,12,1)
% end


%Add label
mi_clearselected();
mi_addblocklabel(0,D_stator_outer*0.5-1);
mi_selectlabel(0,D_stator_outer*0.5-1);
mi_setblockprop('M-19 Steel',0, 1, '<None>', 0, 0, 0);

% mi_clearselected();
% mi_addblocklabel(0,D_rotor_inner*0.5-1);
% mi_selectlabel(0,D_rotor_inner*0.5-1);
% mi_setblockprop('M-19 Steel', 0, 1, 'None', 0, 12, 0);

mi_clearselected();
mi_addblocklabel(0,D_stator_outer*0.5+1);
mi_selectlabel(0,D_stator_outer*0.5+1);
mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);

mi_clearselected();
mi_addblocklabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_selectlabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);

mi_clearselected();
mi_addblocklabel(0,0);
mi_selectlabel(0,0);
% mi_setblockprop('Air', 1, 0, '<None>', 0, 2, 0);
% mi_setblockprop('NoMag',0,1,'None',0,12,1)

mi_setblockprop('M-19 Steel', 0, 1, 'None', 0, 12, 0);

mi_clearselected()
mi_refreshview()

Coil_area=18.08420
ampere_per_density=Num_turns/Coil_area*0.5
% current_max=3.35*0.5
% elec_deg=0
% temp_J_=current_max/ampere_per_density

num_iter=0
% mi_selectcircle(0,0,(D_rotor_outer*0.5+airgap*0.5),4)
% mi_moverotate(0,0,-45)
% mi_selectgroup(12)
% mi_moverotate(0,0,4*180/pole_number/4)
    mi_selectcircle(0,0,(D_rotor_outer*0.5+airgap*0.5),4)
    mi_moverotate(0,0,30)



% ampere_per_density=1
iq=[];
id=[];
current_max=3.35*3;

num_iter=21
for n=1:(num_iter)
    for m=1:1:(num_iter)
        id((n-1)*(num_iter)+m)=-current_max+current_max/num_iter*(m-1)*2;
        iq((n-1)*(num_iter)+m)=-current_max+current_max/num_iter*(n-1)*2;
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
        mo_clearcontour()
    end
    flux__(:,r)=flux
    flux_b=-(flux(1)+flux(2)+flux(6)+flux(7)-flux(8)-flux(9)+flux(10)+flux(11)+flux(15)+flux(16)-flux(17)-flux(18));
    flux_a=-(flux(1)-flux(2)-flux(3)+flux(4)+flux(5)+flux(9)+flux(10)-flux(11)-flux(12)+flux(13)+flux(14)+flux(18));
    flux_c=-(flux(3)+flux(4)-flux(5)-flux(6)+flux(7)+flux(8)+flux(12)+flux(13)-flux(14)-flux(15)+flux(16)+flux(17));
    
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

save('temp.mat')

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


% 
%  for r=1:1:length(flux__)
% 
%     flux_a=flux__(1,r)+flux__(2,r)+flux__(6,r)+flux__(7,r)-flux__(8,r)-flux__(9,r)+flux__(10,r)+flux__(11,r)+flux__(15,r)+flux__(16,r)-flux__(17,r)-flux__(18,r);
%     flux_b=flux__(1,r)-flux__(2,r)-flux__(3,r)+flux__(4,r)+flux__(5,r)+flux__(9,r)+flux__(10,r)-flux__(11,r)-flux__(12,r)+flux__(13,r)+flux__(14,r)+flux__(18,r);
%     flux_c=flux__(3,r)+flux__(4,r)-flux__(5,r)-flux__(6,r)+flux__(7,r)+flux__(8,r)+flux__(12,r)+flux__(13,r)-flux__(14,r)-flux__(15,r)+flux__(16,r)+flux__(17,r);
%     
%     flux_phase_a(r)=flux_a*Num_turns/Num_parallel;
%     flux_phase_b(r)=flux_b*Num_turns/Num_parallel;
%     flux_phase_c(r)=flux_c*Num_turns/Num_parallel;
%  end
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
% 
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
    end
end

for p=1:1:size(flux_d,1)
    diff_flux_d_d(p,:)=diff(flux_d(p,:))./diff(i_d(p,:))*1000;
    diff_flux_q_q(:,p)=diff(flux_q(:,p))./diff(i_q(:,p))*1000;    
    diff_flux_d_q(p,:)=diff(flux_d(:,p))./diff(i_q(:,p))*1000;
    diff_flux_q_d(:,p)=diff(flux_q(p,:))./diff(i_d(p,:))*1000; 
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
Z = diff_flux_d_d;
figure(114)
s = surf(X,Y,Z);
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_d_d[mH]')
% zlim([0 1])
grid on
% title('Variation of q-axis cross-coupling inductance L_q_d')

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = diff_flux_d_q;
figure(117)
s = surf(X,Y,Z);
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_d_q[mH]')
% zlim([0 1])
grid on

[X,Y] = meshgrid(-9.5*iq_step:iq_step:9.5*iq_step,-10*id_step:id_step:10*id_step);
Z = diff_flux_q_q;
figure(115)
s = surf(X,Y,Z');
xlabel('i_q[A]');
ylabel('i_d[A]');
zlabel('L_q_q[mH]')
%  zlim([0 1])
grid on

[X,Y] = meshgrid(-9.5*id_step:id_step:9.5*id_step,-10*iq_step:iq_step:10*iq_step);
Z = diff_flux_q_d;
figure(116)
s = surf(X,Y,Z');
xlabel('i_d[A]');
ylabel('i_q[A]');
zlabel('L_q_d[mH]')
% zlim([0 1])
grid on


%%

%measured 

L_exp=[336.8 348.5 354 352 345 333 327 326 335 350 353 345 331 329 338 351 353 351 338 329 332 341 354 338 329 336]*10^-3*0.5*3/2
figure(4)
plot(L_exp)
