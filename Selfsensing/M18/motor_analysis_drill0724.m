%Ye gu kang
%Drill motor IPM

num_slot=6
num_pole=4

coil_length=2*(155+122)+54+155
length_tot=coil_length
Num_stranded_wire=1
coil_diameter=1.15
coil_turn=9
winding_temperature=25

resistivity_cu=(3.76*winding_temperature+873)*10^(-6)/55

Num_series=2
Num_parallel=1

coil_resistance=resistivity_cu*(length_tot)/(pi*coil_diameter^2/4*Num_stranded_wire)
%theoretical result
R_phase=coil_resistance*Num_series/Num_parallel

0.1/(2*R_phase)