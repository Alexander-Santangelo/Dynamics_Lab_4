function motor_forces = ComputeMotorForces(Fc, Gc, d, km)


Z_c = Fc(3);

L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);



f_1 = (-(Z_c / 4)) + (-((sqrt(2) * L_c) / (4 * d))) + (((sqrt(2) * M_c) / (4 * d))) + ((N_c / (4 * km)))   ;
f_2 = (-(Z_c / 4)) + (-((sqrt(2) * L_c) / (4 * d))) + (-((sqrt(2) * M_c) / (4 * d))) + (-(N_c / (4 * km)))   ;
f_3 = (-(Z_c / 4)) + (((sqrt(2) * L_c) / (4 * d))) + (-((sqrt(2) * M_c) / (4 * d))) + ((N_c / (4 * km)))    ;
f_4 = (-(Z_c / 4)) + (((sqrt(2) * L_c) / (4 * d))) + (((sqrt(2) * M_c) / (4 * d))) + (-(N_c / (4 * km)))    ;



motor_forces = [f_1, f_2, f_3, f_4];


end













