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

% Calculate trim angle values


% Initial state
var0 = zeros(12,1);
var0(1:3) = 0;
var0(4:6) = [0; 5; 0];
var0(10:12) = 0;

% Drag at 5 m/s
V_trim = var0(4:6);
F_drag = -nu * norm(V_trim) * V_trim;
D = norm(F_drag);

% Trim angles for constant 5 m/s lateral/east motion
phi_trim = -atan2(D, m*g);
theta_trim = 0;
psi_trim = 0;

var0(7:9) = [phi_trim; theta_trim; psi_trim];

% Total thrust needed to balance both weight and drag
T_trim = sqrt((m*g)^2 + D^2);

% Equal motor forces for trim
motor_forces = (T_trim/4) * [1; 1; 1; 1];

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
tspan = [0 10];

odefun = @(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);

[t2, y2] = ode45(odefun, tspan, var0, opts);

% Motion plot
y2_plot = [y2(:,1:3), y2(:,4:6), (y2(:,7:9)), y2(:,10:12)];

% Control Plot
PlotAircraftSim(t2, y2_plot,  1:6, 'b-');

%% Figures & Functions
function PlotAircraftSim(time, aircraft_state_array, fig, col)

n = length(time);

% inertial position
figure(fig(1));
clf;

subplot(3,1,1);
plot(time, aircraft_state_array(:,1), col, 'LineWidth', 1.2);
grid on
ylabel('x_E (m)');
xlabel('Time (s)');
title('Inertial Position vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,2), col, 'LineWidth', 1.2);
grid on
ylabel('y_E (m)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, -aircraft_state_array(:,3), col, 'LineWidth', 1.2);
grid on
ylabel('z_E (m)');
xlabel('Time (s)');


% Euler angles
figure(fig(2));
clf;

subplot(3,1,1);
plot(time, aircraft_state_array(:,7)*180/pi, col, 'LineWidth', 1.2);
grid on
ylabel('\phi (deg)');
xlabel('Time (s)');
title('Euler Angles vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,8)*180/pi, col, 'LineWidth', 1.2);
grid on
ylabel('\theta (deg)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,9)*180/pi, col, 'LineWidth', 1.2);
grid on
ylabel('\psi (deg)');
xlabel('Time (s)');


% body velocity
figure(fig(3));
clf;

subplot(3,1,1);
plot(time, aircraft_state_array(:,4), col, 'LineWidth', 1.2);
grid on
ylabel('u (m/s)');
xlabel('Time (s)');
title('Body Velocity vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,5), col, 'LineWidth', 1.2);
grid on
ylabel('v (m/s)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,6), col, 'LineWidth', 1.2);
grid on
ylabel('w (m/s)');
xlabel('Time (s)');


% angular velocity
figure(fig(4)); clf;

subplot(3,1,1);
plot(time, aircraft_state_array(:,10), col, 'LineWidth', 1.2);
grid on
ylabel('p (rad/s)');
xlabel('Time (s)');
title('Angular Velocity vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,11), col, 'LineWidth', 1.2);
grid on
ylabel('q (rad/s)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,12), col, 'LineWidth', 1.2);
grid on
ylabel('r (rad/s)');
xlabel('Time (s)');


% 3D path
figure(fig(6));
clf;

z_up = -aircraft_state_array(:,3);

plot3(aircraft_state_array(:,1), aircraft_state_array(:,2), z_up, col, 'LineWidth', 1.2);
hold on
plot3(aircraft_state_array(1,1), aircraft_state_array(1,2), z_up(1), 'g*');
plot3(aircraft_state_array(n,1), aircraft_state_array(n,2), z_up(n), 'r*');
grid on
title('3D Path')
xlabel('x_E (m)');
ylabel('y_E (m)');
zlabel('z_E (m)');

% % Control Inputs
% figure(fig(7));
% clf;
% % Plot Control Inputs Zc, Lc, Mc, Nc
% subplot(4,1,1);
% plot(time, control_input_array(:,1), col, 'LineWidth', 1.2);
% grid on
% ylabel('Z_c (N)');
% title('Control Inputs vs Time');
% 
% subplot(4,1,2);
% plot(time, control_input_array(:,2), col, 'LineWidth', 1.2);
% grid on
% ylabel('L_c (N*m)');
% 
% subplot(4,1,3);
% plot(time, control_input_array(:,3), col, 'LineWidth', 1.2);
% grid on
% ylabel('M_c (N*m)');
% 
% subplot(4,1,4);
% plot(time, control_input_array(:,4), col, 'LineWidth', 1.2);
% grid on
% ylabel('N_c (N*m)');
% xlabel('Time (s)');

% Auto Save Figures
save_folder = 'C:\Users\Xander\OneDrive\Homework\Junior_second_sem\Dynamics\Dynamics_Lab\Lab4\figures';

names = {'1.4_position','1.4_euler','1.4_velocity','1.4_angular','1.4_path'};
fig_ids = [fig(1), fig(2), fig(3), fig(4), fig(6)];

for i = 1:length(fig_ids)
    figure(fig_ids(i));
    filename = fullfile(save_folder, sprintf('%s_%s.png', names{i}, col(1)));
    saveas(gcf, filename);
end

end

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
%R_B2I = angle2dcm(psi, theta, phi, 'ZYX');
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

