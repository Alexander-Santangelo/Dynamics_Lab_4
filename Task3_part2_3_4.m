clc;
close all;
clear;

%% Task 3 Stuff
% Parameters
m  = 0.068;   % kg
d  = 0.060;   % m
km = 0.0024;  % N*m/N

Ix = 5.8e-5;  % kg*m^2
Iy = 7.2e-5;  % kg*m^2
Iz = 1.0e-4;  % kg*m^2
I  = diag([Ix, Iy, Iz]);

nu = 1e-3;    % N/(m/s)^2
mu = 2e-6;    % N*m/(rad/s)^2
g  = 9.81;    % m/s^2

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
tspan = [0 10];





%% 3.3 --> Graphs of 3.2 at multiple perturbation states



% Parameters
var0 = zeros(12,1);

% Options
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
tspan = [0 10];

% Choose one case at a time
% Cases:

% a) +5 deg roll
% var0(7) = deg2rad(5);

% b) +5 deg pitch
% var0(8) = deg2rad(5);

% c) +0.1 roll rate
 % var0(10) = 0.1;

% d) +0.1 pitch rate
 var0(11) = 0.1;

% 3.2 ODE45 and Plot
odefun = @(t,var) QuadrotorEOM_Linearized_Innerloop(t,var,g,m,I);
[t,y] = ode45(odefun,tspan,var0,opts);

% Control history
control_input_array = zeros(length(t),4);
for i = 1:length(t)
    [Fc,Gc] = InnerLoopFeedback(y(i,:)');
    control_input_array(i,:) = [Fc(3), Gc(1), Gc(2), Gc(3)];
end

% Linearized Motion plot
y_plot = [y(:,1:3), y(:,4:6), (y(:,7:9)), y(:,10:12)];

% Control Plot
PlotAircraftSim(t, y_plot, control_input_array, 1:7, 'b-');


%%
% 3.4 Plot
% Simulate nonlinear EOM (Task 3.4) and plot alongside linearized results

var0_nl = zeros(12,1);

% Cases:

% a) +5 deg roll
% var0(7) = deg2rad(5);

% b) +5 deg pitch
% var0_nl(8) = deg2rad(5);

% c) +0.1 roll rate
% var0_nl(10) = 0.1;

% d) +0.1 pitch rate
 var0_nl(11) = 0.1;


odefun_nl = @(t,var) QuadrotorEOM_NonLinear_InnerLoop(t,var,g,m,I,d,km,nu,mu);
[t_nl, y_nl] = ode45(odefun_nl, tspan, var0_nl, opts);

control_input_array_nl = zeros(length(t_nl),4);

for i = 1:length(t_nl)
    [Fc_nl, Gc_nl] = InnerLoopFeedback(y_nl(i,:)');
    control_input_array_nl(i,:) = [Fc_nl(3), Gc_nl(1), Gc_nl(2), Gc_nl(3)];
end

y_nl_plot = [y_nl(:,1:3), y_nl(:,4:6), y_nl(:,7:9), y_nl(:,10:12)];
PlotAircraftSim(t_nl, y_nl_plot, control_input_array_nl, 1:7, 'r--');




%% Figures & Functions
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

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

% Control Inputs
figure(fig(7));
clf;
% Plot Control Inputs Zc, Lc, Mc, Nc
subplot(4,1,1);
plot(time, control_input_array(:,1), col, 'LineWidth', 1.2);
grid on
ylabel('Z_c (N)');
title('Control Inputs vs Time');

subplot(4,1,2);
plot(time, control_input_array(:,2), col, 'LineWidth', 1.2);
grid on
ylabel('L_c (N*m)');

subplot(4,1,3);
plot(time, control_input_array(:,3), col, 'LineWidth', 1.2);
grid on
ylabel('M_c (N*m)');

subplot(4,1,4);
plot(time, control_input_array(:,4), col, 'LineWidth', 1.2);
grid on
ylabel('N_c (N*m)');
xlabel('Time (s)');

% Auto Save Figures
save_folder = 'C:\Users\Xander\OneDrive\Homework\Junior_second_sem\Dynamics\Dynamics_Lab\Lab4\figures';

names = {'NLposition4','NLeuler4','NLvelocity4','NLangular4','NLpath4', 'NLcontrol4'};
fig_ids = [fig(1), fig(2), fig(3), fig(4), fig(6),fig(7)];

for i = 1:length(fig_ids)
    figure(fig_ids(i));
    filename = fullfile(save_folder, sprintf('%s_%s.png', names{i}, col(1)));
    saveas(gcf, filename);
end

end

%% additional functions
% Linearized Model (3.3)

function var_dot = QuadrotorEOM_Linearized_Innerloop(t, var, g, m, I)

% Decomposing State Vector 
xE = var(1); 
yE = var(2); 
zE = var(3); 

uE = var(4); 
vE = var(5); 
wE = var(6); 

phi = var(7); 
theta = var(8); 
psi = var(9);

p = var(10); 
q = var(11); 
r = var(12); 

% Decompose I
Ix = I(1,1);  
Iy = I(2,2); 
Iz = I(3,3); 

% Inner Loop
[Fc_lin, Gc_lin] = InnerLoopFeedback(var);

deltaFc = Fc_lin(3) + m*g;  % Fc returned includes -m*g baseline, so add m*g to get delta

% Control moments 
deltaGc = Gc_lin; % Use directly since there is no baseline

Lc = deltaGc(1); 
Mc = deltaGc(2); 
Nc = deltaGc(3); 

% Linearized Model

pdotE = [uE;vE;wE]; 
o_vector = [p;q;r];
vdotEB = g*[-theta;phi;0] + (1/m) * [0;0;deltaFc];
omegadotEB = [(1/Ix)*Lc;(1/Iy)*Mc;(1/Iz)*Nc];

var_dot = [pdotE;vdotEB;o_vector;omegadotEB];

end

%% Innerloop Feedback 3.2 Function

function [Fc, Gc] = InnerLoopFeedback(var)

% State Variable Input = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]
% However, only need angles and angle rates
phi   = var(7);
theta = var(8);

p = var(10);
q = var(11);
r = var(12);

% constant parameters
m = 0.068;
g = 9.81;

Ix = 5.8e-5;
Iy = 7.2e-5;

% Gains from 3.1
k1_roll  = 12 * Ix;
k2_roll  = 20 * Ix;

k1_pitch = 12 * Iy;
k2_pitch = 20 * Iy;

k_r = 0.004;

% Control Force from 3.1
Fc = [0; 0; -m*g];

% Control Moment vector
L_c = -k1_roll*p  - k2_roll*phi;
M_c = -k1_pitch*q - k2_pitch*theta;
N_c = -k_r*r;

Gc = [L_c; M_c; N_c];

end



%% 3.4 Non-Linear EOM with lateral feedback

% same as 3.3 but using our non-linear EOM (add k3)

function var_dot = QuadrotorEOM_NonLinear_InnerLoop(t, var, g, m, I, d, km, nu, mu)
% Nonlinear EOM with inner-loop controller (minimal comments)

% Velocities
u = var(4);
v = var(5);
w = var(6);

% Attitudes and rates
phi = var(7);
theta = var(8);
psi = var(9);

p = var(10);
q = var(11);
r = var(12);

% Rotation matrix to inertial
R_B2I = angle2dcm(phi, theta, psi, 'XYZ');

% Body Velocity in inertial frame
vel_body = [u; v; w];
pos_dot = R_B2I * vel_body;

% Non-linear feedback fxn
[Fc, Gc] = InnerLoopFeedback(var);

% Convert to individual motor thrusts
motor_forces = ComputeMotorForces(Fc, Gc, d, km);
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

% Simple aerodynamic drag
Va = norm(vel_body);
if Va == 0
    F_aero = [0;0;0];
else
    F_aero = -nu * Va * vel_body;
end

% Thrust Force
Fc_body = -(f1 + f2 + f3 + f4);
F_body = [0; 0; Fc_body];

% Gravity Force
g_inertial = [0; 0; m*g];
F_gravity_body = R_B2I' * (-g_inertial);

% Translational dynamics
F_tot_B = F_body + F_gravity_body + F_aero;
omega_B = [p; q; r];
vel_dot = (1/m) * F_tot_B - cross(omega_B, vel_body);

% Euler kinematics
T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
euler_dot = T * omega_B;

% Moments from rotors
L = d/sqrt(2) * (-f1 - f2 + f3 + f4);
M = d/sqrt(2) * ( f1 - f2 - f3 + f4);
N = km * (f1 - f2 + f3 - f4);

% Rotational aerodynamic damping
omega_mag = norm(omega_B);
if omega_mag == 0
    M_aero = [0;0;0];
else
    M_aero = -mu * omega_mag * omega_B;
end

% Rotational dynamics
moments = [L; M; N] + M_aero;
omega_B_dot = I \ (moments - cross(omega_B, I*omega_B));

% Pack state derivative
var_dot = zeros(12,1);
var_dot(1:3)   = pos_dot;
var_dot(4:6)   = vel_dot;
var_dot(7:9)   = euler_dot;
var_dot(10:12) = omega_B_dot;
end
function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

Z_c = Fc(3);

L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);

f_1 = -(Z_c/4) - (sqrt(2)*L_c)/(4*d) + (sqrt(2)*M_c)/(4*d) + N_c/(4*km);
f_2 = -(Z_c/4) - (sqrt(2)*L_c)/(4*d) - (sqrt(2)*M_c)/(4*d) - N_c/(4*km);
f_3 = -(Z_c/4) + (sqrt(2)*L_c)/(4*d) - (sqrt(2)*M_c)/(4*d) + N_c/(4*km);
f_4 = -(Z_c/4) + (sqrt(2)*L_c)/(4*d) + (sqrt(2)*M_c)/(4*d) - N_c/(4*km);

motor_forces = [f_1; f_2; f_3; f_4];

end