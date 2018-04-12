%Torque
close all
clear all

load 'C:\Users\a0232593\Desktop\TI_0725\Motor_Inductance_study\drill\Results-inductance7.txt'


torque=Results_inductance7(:,8)
elec_angle=0:10:360

figure(1)
plot(elec_angle,torque)
%%
K_e=1/(200/60*2*pi)
K_t=K_e*3
K_t*10