% Contributors: Xander Santangelo
% Course number: ASEN 3801
% File name: Quadrotor_Linearized.m
% Created: 3/3/2026


function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

% Unpack state vector in 12 state variablees
% var = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
pN = var(1);
pE = var(2); 
pD = var(3); 

u  = var(4);
v  = var(5);
w  = var(6);

phi   = var(7);
theta = var(8);
psi   = var(9);

p = var(10);
q = var(11);
r = var(12);



% DCM to Inertial (NED)

R_B2I = angle2dcm(phi. theta, psi, 'XYZ');

% Translational Position rates (velocity to inertial frame)
vel_body = [u; v; w];
pos_dot = R_B2I * vel_body;

%% Forces

%motor_forces are [f1; f2; f3; f4] along z hat
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

% Total thrust

Fc_body = -(f1 + f2 + f3 + f4);
F_body = [0; 0; Fc_body];

% Gravity in inertial frame to body frame

g_inertial = [0; 0; m*g]; 
F_gravity_body = R_ib' * (-g_inertial); % gravity force to body frame

% Total body forces

F_tot_B = F_body + F_gravity_body;

% Accelerations in body frame
omega_B = [p; q; r];
vel_dot = (1/m) * F_tot_B - cross(omega, vel_body);


T = [1, sphi*sth/cth, cphi*sth/cth;
     0, cphi,         -sphi;
     0, sphi/cth,     cphi/cth];
euler_dot = T * omega;

% Moments

L = d/sqrt(2) * (-f1 - f2 + f3 +f4);
M = d/sqrt(2) * (f1 - f2 - f3 +f4);
N = d/sqrt(2) * (f1 - f2 + f3 - f4);

moments = [L; M; N];

% Angular accelerations
I_cross = cross(omega_B,I);
omega_B_dot = I_cross + I^-1 * moments + I^-1*moments;

% Assemble var_dot
var_dot = zeros(12,1);
var_dot(1:3) = pos_dot;
var_dot(4:6) = vel_dot;
var_dot(7:9) = euler_dot;
var_dot(10:12) = omega_B_dot;

end

