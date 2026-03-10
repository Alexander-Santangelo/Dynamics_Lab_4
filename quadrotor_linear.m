% Contributors: Xander Santangelo
% Course number: ASEN 3801
% File name: Quadrotor_Linearized.m
% Created: 3/3/2026

% Parameters
m  = 0.068; % kg
d  = 0.060;% m
km = 0.0024; % N*m per N (control moment coefficient)
Ix = 5.8e-5; % kg*m^2
Iy = 7.2e-5; % kg*m^2
Iz = 1.0e-4; % kg*m^2
I  = diag([Ix, Iy, Iz]);

nu = 1e-3; % Aerodynamic force coefficient (N/(m/s)^2)
mu = 2e-6; % Aerodynamic moment coefficient (N*m/(rad/s)^2)

g = 9.81; % gravity (m/s^2)

% % hover approximate and small perturbation
% f_hover = m * g / 4;
% motor_forces = f_hover * ones(4, 1); motor_forces(1) = motor_forces(1) * 1;
% motor_forces(3) = motor_forces(3) * 1;
% 
% % Initial state:
% var0 = zeros(12, 1);
% var0(1:3) = 0.0;
% var0(7) = deg2rad(0);
% var0(8) = deg2rad(0);
% var0(9) = deg2rad(0);
% var0(10:12) = deg2rad([0; 0; 0]);
% 
% opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
% tspan = [0 10];
% 
% odefun = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
% 
% [t, y] = ode45(odefun, tspan, var0, opts);
% % Plots
% figure
% plot3(y(:,1), y(:,2), y(:,3), 'LineWidth',1.5)
% grid on
% xlabel('North (m)')
% ylabel('East (m)')
% zlabel('Down (m)')
% title('Quadrotor Position')
% axis equal

% Hover motor forces
f_hover = m * g / 4;
motor_forces = f_hover * zeros(4,1);


% Initial state: hover at origin, zero velocities and angles
var0 = zeros(12,1);
var0(1:3) = 0;            % pn, pe, pd
var0(4:6) = 0;            % u, v, w
var0(7:9) = 0;            % phi, theta, psi
var0(10:12) = 0;          % p, q, r

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
tspan = [0 10];

odefun = @(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);

[t, y] = ode45(odefun, tspan, var0, opts);

% Plot position over time (N,E,D)
figure;
plot3(y(:,1), y(:,2), y(:,3), 'LineWidth', 1.5);
grid on;
xlabel('North (m)');
ylabel('East (m)');
zlabel('Down (m)');
title('Quadrotor Position (Hover)');
axis equal;

%%
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

R_B2I = angle2dcm(phi, theta, psi, 'XYZ');
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
F_gravity_body = R_B2I' * (g_inertial); % gravity force to body frame

% Drag force calculation
F_drag = -nu * norm(vel_body) * vel_body;

% Total body forces

% No drag
% F_tot_B = F_body + F_gravity_body;

F_tot_B = F_body + F_gravity_body + F_drag;



% Accelerations in body frame
% L = d/sqrt(2) * (-f1 - f2 + f3 + f4);
% M = d/sqrt(2) * (f1 - f2 - f3 + f4);
% N = d/sqrt(2) * (f1 - f2 + f3 - f4);

omega_B = [p; q; r];
vel_dot = (1/m) * F_tot_B - cross(omega_B, vel_body);


T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

euler_dot = T * omega_B;

% Moments

Lc = d/sqrt(2) * (-f1 - f2 + f3 +f4);
Mc = d/sqrt(2) * (f1 - f2 - f3 +f4);
Nc = km * (f1 - f2 + f3 - f4);

control_moments = [Lc; Mc; Nc];

% Angular accelerations

omega_B_dot = I \ (control_moments - cross(omega_B, I*omega_B));

% Assemble var_dot
var_dot = zeros(12,1);
var_dot(1:3) = pos_dot;
var_dot(4:6) = vel_dot;
var_dot(7:9) = euler_dot;
var_dot(10:12) = omega_B_dot;

end

