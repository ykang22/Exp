clear all;close all;clc;


Num_slot=12;
Num_pole=8;
Num_series=4;
Num_parallel=1;
Num_turns=25;
pole_pair_num=Num_pole/2;

V_pk=14.36;
Hz=66.3;

K_Back_emf_Hz=V_pk/2/Hz;
K_Back_emf_rad=K_Back_emf_Hz*(Num_pole/2)/(2*pi)
K_t=K_Back_emf_rad*3/2

%awg 20
% with insulation
coil_diameter_w_insu=0.878;
%without insulation
coil_diameter=0.8118;
%coil
coil_depth=26;
coil_width=5;
coil_f_radius=0.5;

Nxt=floor((38.6-25.4)/coil_diameter_w_insu);
Nzt=ceil(Num_turns/Nxt);

winding_temperature=25;
resistivity_cu=(3.76*winding_temperature+873)*10^(-6)/55;
Nxt(1)=15;
Nxt(2)=11;
%*1.05 is a factor to consider increasing winding length for crossing
%winding
for n=1:1:Nzt
    length_turn=2*(coil_depth+coil_width)*(coil_f_radius+coil_diameter_w_insu/2+3^0.5/2*coil_diameter_w_insu*(n-1))*1.15;
    length_coil(n)=length_turn*(Nxt(n)-(1+(-1)^n)/2);
end
length_tot=sum(length_coil);
coil_resistance=resistivity_cu*(length_tot)/(pi*coil_diameter^2/4);
%theoretical result
R_phase=coil_resistance*Num_series/Num_parallel;
%experimental result
% R_phase_exp=0.316

D_stator_outer=85;
D_rotor_outer=47;
airgap=0.5;
%measured value from CAD
slot_area=160/2;

% slot_area=74;

slot_area=78;

current_density_ratio=slot_area/((pi*coil_diameter^2/4)*Num_turns);
ampere_per_density=current_density_ratio*(pi*coil_diameter^2/4);


% base frequency in Hz
wbase=4000/60; %(4000 rev/minute)*(minute/(60*seconds))

% range of speeds over which to evaluate losses
SpeedMin = 100; % in RPM
SpeedMax = 8000; % in RPM
SpeedStep = 100; % in RPM

% Winding properties
MyIdCurrent = 0; % direct current in phase current amplitude scaling
MyIqCurrent = 2; % quadrature current phase current amplitude scaling
MyLowestHarmonic = 2; % lowest numbered harmonic present in the stator winding

AWG=20;	% Magnet wire gauge used in winding
WindingFill=0.3882;
PhaseResistance = R_phase; %0.223*2.333; % phase resistance including end turns at 20 degC
TemperatureRise = 100; % temperature increase, degrees C

% Magnet properties
RotorMagnets = Num_pole;
omag = 0.05*10^6;					% conductivity of sintered NdFeB in S/m

% Core properties
ce = 0.530; % Eddy current coefficient in (Watt/(meter^3 * T^2 * Hz^2)
ch = 143.; % Hysteresis coefficient in (Watts/(meter^3 * T^2 * Hz)
cs = 0.95;  % Lamination stacking factor (nondimensional)
%%%%%%%%%%%

% helpful unit definitions
PI=pi; Pi=pi;
deg=Pi/30.;

% angle through which to spin the rotor in degrees
n_deg = 360/MyLowestHarmonic;

% angle increment in degrees
dk = 1;

% A similar loss/volume expression can be derived to compute proximity
% effect losses in the windings.  Use the low frequency approximiation
% from the paper, since in this case, wire size is a lot smaller than skin
% depth at the frequencies of interest.

% Get parameters for proximity effect loss computation for phase windings
dwire=0.324861*0.0254*exp(-0.115942*AWG); % wire diameter in meters as a function of AWG
owire = (58.*10^6)/(1+TemperatureRise*0.004); % conductivity of the wire in S/m at prescribed deltaT
cePhase = (Pi^2/8)*dwire^2*WindingFill*owire;


openfemm
newdocument(0)
mi_saveas('C:/femm42/temp1.FEM')
path='C:/femm42/'
name_fem='ionloss1.fem'
mi_saveas([path,name_fem])

%add materials

Hc=690000*0.67
idensity = 0
u_r_mag=1.25

electrical_resistive=20%¥ì§Ùm 
electrical_conductivity=1/(electrical_resistive*10^-6)/10^6 %MS/m
mi_addmaterial('NdFeB 10 MGOe (Bonded)',u_r_mag,u_r_mag,Hc,0,electrical_conductivity)
mi_getmaterial('Air')
mi_getmaterial('M-19 Steel');
mi_getmaterial('Pure Iron')
mi_addmaterial('NoMag',u_r_mag,u_r_mag,0,0,electrical_conductivity)
mi_addmaterial('a+', 1, 1, 0, -idensity/2, 56)
mi_addmaterial('a-', 1, 1, 0, idensity/2, 56)
mi_addmaterial('b+', 1, 1, 0, idensity, 56)
mi_addmaterial('b-', 1, 1, 0, -idensity, 56)
mi_addmaterial('c+', 1, 1, 0, -idensity/2, 56)
mi_addmaterial('c-', 1, 1, 0, idensity/2, 56)
mi_addboundprop('A_0',0,0,0,0,0,0,0,0,0)

%add parameter
depth = 25.4*0.96
pole_number= 8
slot_number = 12
D_stator_outer =  42.544*2
D_stator_inner=47.75
w_teeth = 4.826
th_core = 4.189
airgap = 0.381
D_rotor_outer=47
D_rotor_inner = 39
m_thickness=3.82
D_shaft=14.2875*2*0.7
sue_1=1.5
sue_2=1.1
R_sue_3=D_rotor_outer*0.5+airgap
R_sue_2=R_sue_3+sue_2
R_sue_1=R_sue_2+sue_1
slot_ratio = 0.85
pole_arc_ratio=0.80

%winding layout
q_denominator=pole_number*3/gcd(slot_number,pole_number*3)
q_numerator=slot_number/gcd(slot_number,pole_number*3)
num_zeros=q_denominator-q_numerator
seq_zero= num_zeros/q_numerator
sequence=[]
num_repeat=slot_number/q_numerator
temp_num_zeros=ceil(seq_zero-0.5)
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
    end
end

seq=[]
Dist=[-12,11, -10, 12, -11, 10]
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
phase_shift=-pi/2
E_u=[]
E_v=[]
E_w=[]
for x=1:length(S1_u)
    E_u(x)=exp(1i*(abs(S1_u(x))*pi*pole_number/(slot_number)+phase_shift))*S1_u(x)/abs(S1_u(x))/2*3/slot_number  ;
    E_v(x)=exp(1i*(abs(S1_v(x))*pi*pole_number/(slot_number)+phase_shift))*S1_v(x)/abs(S1_v(x))/2*3/slot_number  ;
    E_w(x)=exp(1i*(abs(S1_w(x))*pi*pole_number/(slot_number)+phase_shift))*S1_w(x)/abs(S1_w(x))/2*3/slot_number  ;
end

E_u_real_old=0
E_u_imag_old=0
E_v_real_old=0
E_v_imag_old=0
E_w_real_old=0
E_w_imag_old=0
for x=1:length(E_u)
    figure(1)
    hold on
    grid on
    axis square
    axis([-1 1 -1 1])
    quiver(E_u_real_old,E_u_imag_old,real(E_u(x)),imag(E_u(x)),'AutoScale', 'on','Color','r','lineWidth',1,'MaxHeadSize',4)
    E_u_real_old=E_u_real_old+real(E_u(x))
    E_u_imag_old=E_u_imag_old+imag(E_u(x))
    quiver(E_v_real_old,E_v_imag_old,real(E_v(x)),imag(E_v(x)),'AutoScale', 'on','Color','b','lineWidth',1,'MaxHeadSize',4)
    E_v_real_old=E_v_real_old+real(E_v(x))
    E_v_imag_old=E_v_imag_old+imag(E_v(x))
    quiver(E_w_real_old,E_w_imag_old,real(E_w(x)),imag(E_w(x)),'AutoScale', 'on','Color','g','lineWidth',1,'MaxHeadSize',4)
    E_w_real_old=E_w_real_old+real(E_w(x))
    E_w_imag_old=E_w_imag_old+imag(E_w(x))
end
WF=abs(sum(E_v))
mi_probdef(0,'millimeters','planar',1e-008,depth,30)

%square for the boundary
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
%core
mi_addnode(-(D_stator_outer*0.5-th_core)*cos(pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core)))),(D_stator_outer*0.5-th_core)*sin(pi/2-(asin(w_teeth*0.5/(D_stator_outer*0.5-th_core)))))
mi_addnode(0,R_sue_3)
mi_addnode(R_sue_3*cos(pi/2+pi/slot_number*slot_ratio),R_sue_3*sin(pi/2+pi/slot_number*slot_ratio))
mi_addnode(R_sue_2*cos(pi/2+pi/slot_number*slot_ratio),R_sue_2*sin(pi/2+pi/slot_number*slot_ratio))
mi_addnode(-R_sue_1*cos(pi/2-(asin(w_teeth*0.5/R_sue_1))),R_sue_1*sin(pi/2-asin(w_teeth*0.5/R_sue_1)))
mi_addsegment(-(D_stator_outer*0.5-th_core)*cos(pi/2-asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))),(D_stator_outer*0.5-th_core)*sin(pi/2-asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))),-R_sue_1*cos(pi/2-asin(w_teeth*0.5/R_sue_1)),R_sue_1*sin(pi/2-asin(w_teeth*0.5/R_sue_1)))
mi_addsegment(-R_sue_1*cos(pi/2-asin(w_teeth*0.5/R_sue_1)),R_sue_1*sin(pi/2-asin(w_teeth*0.5/R_sue_1)),R_sue_2*cos(pi/2+pi/slot_number*slot_ratio),R_sue_2*sin(pi/2+pi/slot_number*slot_ratio))
mi_addsegment(R_sue_2*cos(pi/2+pi/slot_number*slot_ratio),R_sue_2*sin(pi/2+pi/slot_number*slot_ratio),R_sue_3*cos(pi/2+pi/slot_number*slot_ratio),R_sue_3*sin(pi/2+pi/slot_number*slot_ratio))
mi_addarc(0,R_sue_3,R_sue_3*cos(pi/2+pi/slot_number*slot_ratio),R_sue_3*sin(pi/2+pi/slot_number*slot_ratio),(180/slot_number*slot_ratio),5)
mi_addnode((D_stator_outer*0.5-th_core)*cos(pi/2+pi/slot_number),(D_stator_outer*0.5-th_core)*sin(pi/2+pi/slot_number))
mi_addnode(R_sue_2*cos(pi/2+pi/slot_number),R_sue_2*sin(pi/2+pi/slot_number))
mi_addsegment((D_stator_outer*0.5-th_core)*cos(pi/2+pi/slot_number),(D_stator_outer*0.5-th_core)*sin(pi/2+pi/slot_number),R_sue_2*cos(pi/2+pi/slot_number),R_sue_2*sin(pi/2+pi/slot_number))
mi_addsegment(R_sue_2*cos(pi/2+pi/slot_number),R_sue_2*sin(pi/2+pi/slot_number),R_sue_2*cos(pi/2+pi/slot_number*slot_ratio),R_sue_2*sin(pi/2+pi/slot_number*slot_ratio))
mi_addarc(-(D_stator_outer*0.5-th_core)*cos(pi/2-asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))),(D_stator_outer*0.5-th_core)*sin(pi/2-asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))),(D_stator_outer*0.5-th_core)*cos(pi/2+pi/slot_number),(D_stator_outer*0.5-th_core)*sin(pi/2+pi/slot_number),(180/slot_number-180/pi*asin(w_teeth*0.5/(D_stator_outer*0.5-th_core))),5)
mi_selectcircle(0,0,D_stator_outer*0.5,4)
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

%winding
zz=1
for v=1:length(seq)
    xx=floor((v-1)/2)
    zz=zz*-1
    mi_addblocklabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx))
    mi_selectlabel(((D_stator_outer*0.5+R_sue_1-th_core)/2)*cos(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx),((D_stator_outer*0.5+R_sue_1-th_core)/2)*sin(zz*pi/slot_number*0.9+pi/2+2*pi/slot_number*xx))
    if seq(v)==-10
        mi_setblockprop('b-', 1, 0, '<None>', 0,2, 0)
        
%         mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==10
        mi_setblockprop('b+', 1, 0, '<None>', 0, 2, 0)
%         mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==-11
        mi_setblockprop('a-', 1, 0, '<None>', 0, 2, 0)
%         mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==11
        mi_setblockprop('a+', 1, 0, '<None>', 0, 2, 0)
%         mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==-12
        mi_setblockprop('c-', 1, 0, '<None>', 0, 2, 0)
%         mi_setgroup(6)
        mi_clearselected()
    elseif seq(v)==12
        mi_setblockprop('c+', 1, 0, '<None>', 0, 2, 0)
%         mi_setgroup(2)
        mi_clearselected()
    end
end

%magnet
mi_addnode(D_rotor_inner/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_inner/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))
mi_addnode(D_rotor_outer/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_outer/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))
mi_addsegment(D_rotor_inner/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_inner/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_outer/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_outer/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))

mi_addnode(-D_rotor_inner/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_inner/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))
mi_addnode(-D_rotor_outer/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_outer/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))
mi_addsegment(-D_rotor_inner/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_inner/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),-D_rotor_outer/2*cos(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5),D_rotor_outer/2*sin(pi/2+2*pi/pole_number*(1-pole_arc_ratio)*0.5))

mi_selectcircle(0,0,(D_rotor_outer+airgap)*0.5,4)
mi_copyrotate2(0,0,360/pole_number,pole_number,4)
m_group=10
for z=1:(pole_number/2)
    m_group=m_group+1
    mi_clearselected()
    mi_addblocklabel(cos(pi/2+pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2+pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_selectlabel(cos(pi/2+pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2+pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_setblockprop('NdFeB 10 MGOe (Bonded)',0,1,'None',90+180/pole_number+(z-1)*4*180/pole_number,m_group,1)
    m_group=m_group+1
    mi_clearselected()
    mi_addblocklabel(cos(pi/2-pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2-pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_selectlabel(cos(pi/2-pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2-pi/pole_number+(z-1)*4*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_setblockprop('NdFeB 10 MGOe (Bonded)',0,1,'None',-90-180/pole_number+(z-1)*4*180/pole_number,m_group,1)
end

for z=1:pole_number
    m_group=m_group+1
    mi_clearselected()
    mi_addblocklabel(cos(pi/2+(z-1)*2*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2+(z-1)*2*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_selectlabel(cos(pi/2+(z-1)*2*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5,sin(pi/2+(z-1)*2*pi/pole_number)*(D_rotor_inner+m_thickness)*0.5)
    mi_setblockprop('NoMag',0,1,'None',0,m_group,1)
end

x1=0
y1=0
r=D_rotor_inner*0.5
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

x1=0
y1=0
r=D_rotor_outer*0.5
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

x1=0
y1=0
r=D_shaft*0.5
mi_addnode(x1+r,y1)
mi_addnode(x1-r,y1)
mi_addarc(x1+r,y1,x1-r,y1,180,5)
mi_addarc(x1-r,y1,x1+r,y1,180,5)

%Add label
mi_clearselected();
mi_addblocklabel(0,D_stator_outer*0.5-1);
mi_selectlabel(0,D_stator_outer*0.5-1);
mi_setblockprop('M-19 Steel',0, 1, '<None>', 0, 1, 0);

mi_clearselected();
mi_addblocklabel(0,D_rotor_inner*0.5-1);
mi_selectlabel(0,D_rotor_inner*0.5-1);
mi_setblockprop('M-19 Steel', 0, 1, 'None', 0, 10, 0);

mi_clearselected();
mi_addblocklabel(0,D_stator_outer*0.5+1);
mi_selectlabel(0,D_stator_outer*0.5+1);
mi_setblockprop('Air', 1, 0, '<None>', 0, 0, 0);

mi_clearselected();
mi_addblocklabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_selectlabel(0,D_rotor_outer*0.5+airgap*0.5);
mi_setblockprop('Air', 1, 0, '<None>', 0, 0, 0);

mi_clearselected();
mi_addblocklabel(0,0);
mi_selectlabel(0,0);
mi_setblockprop('Air', 1, 0, '<None>', 0, 0, 0);

mi_clearselected()
mi_refreshview()

mi_selectcircle(0,0,(D_rotor_outer*0.5+airgap*0.5),4)
mi_moverotate(0,0,4*180/pole_number/4)
mi_clearselected()

	for m=10:(10+RotorMagnets*2)
        mi_selectgroup(m);
    end

mi_moverotate(0,0,4*180/pole_number/4)
ampere_per_density=3.0760

current_max=10
num_iter=20

iq=[]
id=[]

iq(1)=10.0001
id(1)=-10.0001

temp_J=[]
I =[]
elec_deg=[]

for n=1,length(id)
    I(n)=(iq(n)^2+id(n)^2)^0.5
    temp_J(n)=I(n)/ampere_per_density
    elec_deg(n)=atan2(id(n),iq(n))
end

rot_angle=0;
num_iter=n_deg;
time_passed=0;
flux_phase_a=[];
flux_phase_b=[];
flux_phase_c=[];
for r=1: num_iter-1
    tic
    J_a(r)=temp_J(1)*sin(elec_deg(1)+(rot_angle));
    J_b(r)=temp_J(1)*sin(elec_deg(1)-2*pi/3+(rot_angle));
    J_c(r)=temp_J(1)*sin(elec_deg(1)+2*pi/3+(rot_angle));
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
        
        if seq(v*2)==-10
            flux=mo_lineintegral(0);
            flux_b=flux_b-flux(1);
            mo_clearcontour()
        elseif seq(v*2)==10
            flux=mo_lineintegral(0);
            flux_b=flux_b+flux(1);
            mo_clearcontour()
        elseif seq(v*2)==-11
            flux=mo_lineintegral(0);
            flux_a=flux_a-flux(1);
            mo_clearcontour()
        elseif seq(v*2)==11
            flux=mo_lineintegral(0);
            flux_a=flux_a+flux(1);
            mo_clearcontour()
        elseif seq(v*2)==-12
            flux=mo_lineintegral(0);
            flux_c=flux_c-flux(1);
            mo_clearcontour()
        elseif seq(v*2)==12
            flux=mo_lineintegral(0);
            flux_c=flux_c+flux(1);
            mo_clearcontour()
        end
    end
    flux_phase_a(r)=flux_a*Num_turns*Num_series/Num_parallel/(Num_slot/3);
    flux_phase_b(r)=flux_b*Num_turns*Num_series/Num_parallel/(Num_slot/3);
    flux_phase_c(r)=flux_c*Num_turns*Num_series/Num_parallel/(Num_slot/3);
    
    for m=10:(10+RotorMagnets*2)
        mo_groupselectblock(m);
    end
    T(r) = mo_blockintegral(22)
    Fx(r) = mo_blockintegral(18)
    Fy(r) = mo_blockintegral(19)
    
%     starttime=clock;
    kk = r/dk + 1;
    
    if (r == 1)
        % Record the initial mesh elements if the first time through the loop
        nn = mo_numelements;
        b = zeros(floor(n_deg/dk),nn); % matrix that will hold the flux density info
        A = zeros(floor(n_deg/dk),nn); % matrix that will hold the vector potential info
        z = zeros(nn,1);
        a = zeros(nn,1);
        g = zeros(nn,1);
        tq = zeros(floor(n_deg/dk),1);
        for m = 1:nn
            elm = mo_getelement(m);
            % z is a vector of complex numbers that represents the location of
            % the centroid of each element.  The real part is x, the
            % imaginary part is y.  The purpose of representing the
            % location in this way is that it is easy to rotate the
            % location of the centroid (by multiplying by a complex number)
            % to find the point that corresponds to the centroid when the
            % rotor is rotated.
            z(m) = elm(4) + 1j*elm(5);
            % element area in the length units used to draw the geometry
            a(m) = elm(6);
            % group number associated with the element
            g(m) = elm(7);
        end
    end
    
    % Store element flux densities *)
    u=exp(1j*r*pi/180.);
    for m = 1:nn
        % Element is on the rotor.  Elements on the rotor have been
        % assigned to groups 10 and higher (by assigning the block label that marks them
        % to group 10, 11, 12, etc.) so that the program can tell if an element is part of the rotor.
        if(g(m)>=10)
            % the location in the original mesh is rotated so that the
            % flux density at the same point with the rotor rotated through
            % angle k can be evaluated. Multiplying by the complex number u
            % does the rotation.
            p = z(m)*u;
            % Flux densities bx and by are evaluated and rolled into a complex number.
            % Dividing by the complex number u rotates the flux density
            pv = mo_getpointvalues(real(p),imag(p));
            % back into a rotor-fixed reference frame.
            if (g(m)==10)
                % store flux density for elements in rotor core
                b(kk,m) = (pv(2)+1j*pv(3))/u;
            else
                % store vector potential for elements that are in PMs
                A(kk,m) = pv(1);
            end
        elseif (g(m) > 0) % element is on the stator
            % since the stator doesn't move, no games have to be played
            % with rotations.
            p = z(m);
            b(kk,m) = (mo_getb(real(p),imag(p))*[1;1j]);
        end
    end
    
    % mo_getprobleminfo returns, among other things, the depth of the
    % machine in the into-the-page direction and the length units used to
    % draw the geometry. Both of these pieces of information will be needed
    % to integrate the losses over the volume of the machine.
    probinfo=mo_getprobleminfo;
    
    % select all blocks on the rotor and compute the torque

    
    mo_close()    
    
    mi_clearselected()
    mi_selectcircle(0,0,(D_rotor_outer*0.5+airgap*0.5),4)
    mi_moverotate(0,0,-4*180/pole_number/num_iter)
    
    %     mi_clearselected()
    %     mi_selectgroup(12)
    %     mi_moverotate(0,0,-4*180/pole_number/num_iter)
    rot_angle=2*pi/num_iter+rot_angle
    
    one_iter=toc
    time_require=one_iter*num_iter
    time_passed=one_iter+time_passed
    time_left=time_require-time_passed
end


%% Add Up Core Losses
% close all
load iq10id0.mat
Torque=mean(T)
K_t=0.103
% iq(1)=5
% Torque=iq(1)*K_t
RotorMagnets = Num_pole;
omag = 1/30*10^6;
n = 360/MyLowestHarmonic
% Compute the square of the amplitude of each harmonic at the centroid of
% each element in the mesh. Matlab's built-in FFT function makes this easy.
ns=n/dk;
bxfft=abs(fft(real(b)))*(2/ns);
byfft=abs(fft(imag(b)))*(2/ns);
bsq=(bxfft.*bxfft) + (byfft.*byfft);

% Compute the volume of each element in units of meter^3
h = probinfo(3);            % Length of the machine in the into-the-page direction
lengthunits = probinfo(4);  % Length of drawing unit in meters
v = a*h*lengthunits^2;

% compute fft of A at the center of each element
Jm=fft(A)*(2/ns);
for k=1:RotorMagnets*2
    g3=(g==(10+k));
    % total volume of the magnet under consideration;
    vmag=v'*g3;
    % average current in the magnet for each harmonic
    Jo=(Jm*(v.*g3))/vmag;
    % subtract averages off of each each element in the magnet
    Jm = Jm - Jo*g3';
end
PhaseResistance=R_phase;
TemperatureRise=25
Iphase=sqrt(iq(1)^2+id(1)^2)/sqrt(2);
PhaseOhmic = 3*(PhaseResistance*(1+TemperatureRise*0.004))*Iphase^2

results=[];

for thisSpeed=SpeedMin:SpeedStep:SpeedMax
    
    thisFrequency = thisSpeed/60; % mechanical speed in Hz
    
    % Make a vector representing the frequency associated with each harmonic
    % The last half of the entries are zeroed out so that we don't count each
    % harmonic twice--the upper half of the FFT a mirror of the lower half
    w=0:(ns-1);
    w=MyLowestHarmonic*thisFrequency*w.*(w<(ns/2));
    
    % Now, total core loss can be computed in one fell swoop...
    % Dividing the result by cs corrects for the lamination stacking factor
    g1=(g==10);
    rotor_loss = ((ch*w+ce*w.*w)*bsq*(v.*g1))/cs;
    
    g2=(g==1);
    stator_loss = ((ch*w+ce*w.*w)*bsq*(v.*g2))/cs;
    
    % and prox losses can be totalled up in a similar way
    g4=(g==2);
    prox_loss = ((cePhase*w.*w)*bsq*(v.*g4));
    
    % Add up eddy current losses in the magnets
    magnet_loss = (1/2)*((omag*(2*pi*w).^2)*(abs(Jm).^2)*v);
    
    total_loss = rotor_loss + stator_loss + prox_loss + PhaseOhmic + magnet_loss;
    
    results = [results; thisSpeed, rotor_loss, stator_loss, magnet_loss, PhaseOhmic, prox_loss, total_loss];
end

% Loss plot

figure(29)
plot(results(:,1),results(:,2:7));
xlabel('Speed[RPM]');
ylabel('[W]');
title('Loss versus Speed');
legend('Rotor Core','Stator Core','Magnets','Coil Ohmic','Coil Proximity','Total Loss','Location','northwest');
grid on
 ylim([0 350])
% xlim([0 4000])
figure(21)
P=Torque*results(:,1)*2*pi/60;
eff=(P)./(P+results(:,7))*100
plot(results(:,1),eff)
ylim([0 110])
grid on
xlabel('Speed[RPM]')
ylabel('efficiency[%]')
% xlim([0 4000])
figure(20)
plot(results(:,1),results(:,2:7));
hold on
plot(results(:,1),P)
xlabel('Speed[RPM]');
ylabel('[W]');
title('Loss versus Speed');
legend('Rotor Core','Stator Core','Magnets','Coil Ohmic','Coil Proximity','Total Loss','Output Power','Location','northwest');
grid on
 ylim([0 1000])


wbase=4000/60; %(4000 rev/minute)*(minute/(60*seconds))
w=0:(ns-1);
w=MyLowestHarmonic*wbase*w.*(w<(ns/2)); 

% Rotor loss
g1=(g==10);
ptloss = (transpose ((ch*w+ce*w.*w)*bsq).*g1)/cs;

% Stator loss
g2=(g==1);
ptloss = ptloss + (transpose ((ch*w+ce*w.*w)*bsq).*g2)/cs;

% Prox loss
g4=(g==2);
ptloss = ptloss + (transpose ((cePhase*w.*w)*bsq).*g4);

% PM contribution
ptloss = ptloss + ((1/2)*(omag*(2*pi*w).^2)*(abs(Jm).^2))';

% point location in z
heating = [real(z),imag(z),ptloss];
% save('c:\\femm42\\myLossData','heating','-ASCII');
% clear all

%%
P_max=max(ptloss)
% P_max = 9.2467e+06
% load 'myLossData.txt'
x=heating(:,1);
y=heating(:,2);
z=heating(:,3)/P_max;


% Little triangles
% The solution is to use Delaunay triangulation. Let's look at some
% info about the "tri" variable.
tri = delaunay(x,y);
figure(11)
plot(x,y,'.')
[r,c] = size(tri);
disp(r)
figure(12)
% Plot it with TRISURF
h = trisurf(tri, x, y, z);
axis vis3d
view(2)
% l = light('Position',[-50 -15 29])
lighting phong
shading interp
colorbar EastOutside
