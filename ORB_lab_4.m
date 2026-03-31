clear;
clc;
close all;

%% Parameters
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




%% Hover trim motor forces
f_hover = m*g/4;
motor_forces_hover = f_hover * ones(4,1);




%% Initial state
% State order:
% var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]

var0 = zeros(12,1);

% =========================
% Choose ONE Task 2 case:
% =========================

% 2.1(d) / 2.5 roll rate deviation
% var0(10:12) = [0.1; 0; 0];

% 2.1(e) / 2.5 pitch rate deviation
% var0(10:12) = [0; 0.1; 0];

% 2.1(f) / 2.5 yaw rate deviation
 var0(10:12) = [0; 0; 0.1];





%% Problem 2.3 check: controller output
[Fc_test, Gc_test] = RotationDerivativeFeedback(var0, m, g);

disp('Problem 2.3 controller output:')
disp('Fc = ')
disp(Fc_test)
disp('Gc = ')
disp(Gc_test)




%% Problem 2.4 check: motor forces from control
motor_forces_test = ComputeMotorForces(Fc_test, Gc_test, d, km);

disp('Problem 2.4 motor forces:')
disp(motor_forces_test)





%% Problem 2.5 simulations

% Uncontrolled nonlinear system
odefun_un = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_hover);
[t_un, y_un] = ode45(odefun_un, tspan, var0, opts);

% Controlled nonlinear system with rate feedback
odefun_con = @(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, d, km, nu, mu);
[t_con, y_con] = ode45(odefun_con, tspan, var0, opts);






%% Reformat for PlotAircraftSim
% PlotAircraftSim expects:
% [x y z phi theta psi u v w p q r]

y_un_plot  = [y_un(:,1:3),  rad2deg(y_un(:,7:9)),  y_un(:,4:6),  y_un(:,10:12)];
y_con_plot = [y_con(:,1:3), rad2deg(y_con(:,7:9)), y_con(:,4:6), y_con(:,10:12)];

PlotAircraftSim(t_un,  y_un_plot,  0, [1:6], 'b-');
PlotAircraftSim(t_con, y_con_plot, 0, [1:6], 'r--');






%% Motor forces for controlled and uncontrolled systems
motor_forces_un  = zeros(length(t_un),4);
motor_forces_con = zeros(length(t_con),4);

for i = 1:length(t_un)
    motor_forces_un(i,:) = motor_forces_hover';
end

for i = 1:length(t_con)
    [Fc_con, Gc_con] = RotationDerivativeFeedback(y_con(i,:)', m, g);
    motor_forces_con(i,:) = ComputeMotorForces(Fc_con, Gc_con, d, km)';
end



figure
plot(t_un, motor_forces_un(:,1),  'b-',  'LineWidth', 1.2); hold on
plot(t_un, motor_forces_un(:,2),  'b--', 'LineWidth', 1.2);
plot(t_un, motor_forces_un(:,3),  'b:',  'LineWidth', 1.2);
plot(t_un, motor_forces_un(:,4),  'b-.', 'LineWidth', 1.2);

plot(t_con, motor_forces_con(:,1), 'r-',  'LineWidth', 1.2);
plot(t_con, motor_forces_con(:,2), 'r--', 'LineWidth', 1.2);
plot(t_con, motor_forces_con(:,3), 'r:',  'LineWidth', 1.2);
plot(t_con, motor_forces_con(:,4), 'r-.', 'LineWidth', 1.2);



grid on
xlabel('Time (s)')
ylabel('Motor Force (N)')
title('Problem 2.5 Motor Forces: Uncontrolled vs Controlled')
legend('f1 uncontrolled','f2 uncontrolled','f3 uncontrolled','f4 uncontrolled', ...
       'f1 controlled','f2 controlled','f3 controlled','f4 controlled')







%% FUNCTIONS


function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

p = var(10);
q = var(11);
r = var(12);

Fc = [0; 0; -m*g];

k = 0.004;

Gc = [-k*p; -k*q; -k*r];

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






function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

% State order:
% var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]

xE = var(1);
yE = var(2);
zE = var(3);

u = var(4);
v = var(5);
w = var(6);

phi   = var(7);
theta = var(8);
psi   = var(9);

p = var(10);
q = var(11);
r = var(12);

R_B2I = angle2dcm(phi, theta, psi, 'XYZ');

% Translational position rates
vel_body = [u; v; w];
pos_dot = R_B2I * vel_body;

% Motor forces
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

% Aerodynamic drag forces
Va = norm(vel_body);
if Va == 0
    F_aero = [0; 0; 0];
else
    F_aero = -nu * Va * vel_body;
end

% Total control thrust
Fc_body = -(f1 + f2 + f3 + f4);
F_control = [0; 0; Fc_body];

% Gravity in body frame
g_inertial = [0; 0; m*g];
F_gravity_body = R_B2I' * g_inertial;

% Total body force
F_total = F_control + F_gravity_body + F_aero;

omega_B = [p; q; r];
vel_dot = (1/m) * F_total - cross(omega_B, vel_body);

% Euler angle rates
T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

euler_dot = T * omega_B;

% Control moments
L_c = d/sqrt(2) * (-f1 - f2 + f3 + f4);
M_c = d/sqrt(2) * ( f1 - f2 - f3 + f4);
N_c = km * (f1 - f2 + f3 - f4);

% Aerodynamic moments
omega_mag = norm(omega_B);
if omega_mag == 0
    M_aero = [0; 0; 0];
else
    M_aero = -mu * omega_mag * omega_B;
end

moments = [L_c; M_c; N_c] + M_aero;

omega_B_dot = I \ (moments - cross(omega_B, I*omega_B));

var_dot = zeros(12,1);
var_dot(1:3)   = pos_dot;
var_dot(4:6)   = vel_dot;
var_dot(7:9)   = euler_dot;
var_dot(10:12) = omega_B_dot;

end






function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, d, km, nu, mu)

% State order:
% var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]

xE = var(1);
yE = var(2);
zE = var(3);

u = var(4);
v = var(5);
w = var(6);

phi   = var(7);
theta = var(8);
psi   = var(9);

p = var(10);
q = var(11);
r = var(12);

R_B2I = angle2dcm(phi, theta, psi, 'XYZ');

% Translational position rates
vel_body = [u; v; w];
pos_dot = R_B2I * vel_body;

% Feedback controller
[Fc, Gc] = RotationDerivativeFeedback(var, m, g);
motor_forces = ComputeMotorForces(Fc, Gc, d, km);

f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

% Aerodynamic drag forces
Va = norm(vel_body);
if Va == 0
    F_aero = [0; 0; 0];
else
    F_aero = -nu * Va * vel_body;
end

% Total control thrust
Fc_body = -(f1 + f2 + f3 + f4);
F_control = [0; 0; Fc_body];

% Gravity in body frame
g_inertial = [0; 0; m*g];
F_gravity_body = R_B2I' * g_inertial;

% Total body force
F_total = F_control + F_gravity_body + F_aero;

omega_B = [p; q; r];
vel_dot = (1/m) * F_total - cross(omega_B, vel_body);

% Euler angle rates
T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

euler_dot = T * omega_B;

% Control moments
L_c = d/sqrt(2) * (-f1 - f2 + f3 + f4);
M_c = d/sqrt(2) * ( f1 - f2 - f3 + f4);
N_c = km * (f1 - f2 + f3 - f4);

% Aerodynamic moments
omega_mag = norm(omega_B);
if omega_mag == 0
    M_aero = [0; 0; 0];
else
    M_aero = -mu * omega_mag * omega_B;
end

moments = [L_c; M_c; N_c] + M_aero;

omega_B_dot = I \ (moments - cross(omega_B, I*omega_B));

var_dot = zeros(12,1);
var_dot(1:3)   = pos_dot;
var_dot(4:6)   = vel_dot;
var_dot(7:9)   = euler_dot;
var_dot(10:12) = omega_B_dot;

end







function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

n = length(time);


%% inertial position
figure(fig(1));

subplot(3,1,1);
plot(time, aircraft_state_array(:,1), col); hold on;
ylabel('x_E (m)');
xlabel('Time (s)');
title('Inertial Position vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,2), col); hold on;
ylabel('y_E (m)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,3).*-1, col); hold on;
ylabel('z_E (m)');
xlabel('Time (s)');


%% Euler angles
figure(fig(2));

subplot(3,1,1);
plot(time, aircraft_state_array(:,4), col); hold on;
ylabel('\phi (deg)');
xlabel('Time (s)');
title('Euler Angles vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,5), col); hold on;
ylabel('\theta (deg)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,6), col); hold on;
ylabel('\psi (deg)');
xlabel('Time (s)');


%% inertial velocity in body frame
figure(fig(3));

subplot(3,1,1);
plot(time, aircraft_state_array(:,7), col); hold on;
ylabel('u (m/s)');
xlabel('Time (s)');
title('Body Velocity vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,8), col); hold on;
ylabel('v (m/s)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,9), col); hold on;
ylabel('w (m/s)');
xlabel('Time (s)');


%% angular velocity
figure(fig(4));

subplot(3,1,1);
plot(time, aircraft_state_array(:,10), col); hold on;
ylabel('p (rad/s)');
xlabel('Time (s)');
title('Angular Velocity vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,11), col); hold on;
ylabel('q (rad/s)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,12), col); hold on;
ylabel('r (rad/s)');
xlabel('Time (s)');


%% 3D path
figure(fig(6));

z_up = -aircraft_state_array(:,3);

plot3(aircraft_state_array(:,1), aircraft_state_array(:,2), z_up, col, 'LineWidth', 1.2); hold on
plot3(aircraft_state_array(1,1), aircraft_state_array(1,2), z_up(1), 'g*'); hold on
plot3(aircraft_state_array(n,1), aircraft_state_array(n,2), z_up(n), 'r*'); hold on

grid on
title('3D Path')
xlabel('(m)');
ylabel('(m)');
zlabel('(m)');

end





%% Task 3 Stuff


%% 3.1

% clear;
% clc;
% 
% %% Parameters
% Ix = 5.8e-5;
% Iy = 7.2e-5;
% 
% %% Desired poles
% lambda1 = -2;    % dominant pole -> tau = 0.5 s
% lambda2 = -10;   % faster pole
% 
% %% Desired characteristic polynomial
% % (s - lambda1)(s - lambda2) = s^2 - (lambda1+lambda2)s + lambda1*lambda2
% a1 = -(lambda1 + lambda2);
% a0 = lambda1 * lambda2;
% 
% %% Roll gains
% k1_roll = a1 * Ix;
% k2_roll = a0 * Ix;
% 
% %% Pitch gains
% k1_pitch = a1 * Iy;
% k2_pitch = a0 * Iy;
% 
% %% Yaw rate gain from Task 2
% k_r = 0.004;
% 
% %% Display gains
% disp('Problem 3.1 gains:')
% disp(['k1_roll  = ', num2str(k1_roll)])
% disp(['k2_roll  = ', num2str(k2_roll)])
% disp(['k1_pitch = ', num2str(k1_pitch)])
% disp(['k2_pitch = ', num2str(k2_pitch)])
% disp(['k_r      = ', num2str(k_r)])
% 
% %% Verify roll closed-loop eigenvalues
% A_roll = [0          1;
%          -k2_roll/Ix -k1_roll/Ix];
% 
% eig_roll = eig(A_roll);
% 
% disp('Roll eigenvalues:')
% disp(eig_roll)
% 
% %% Verify pitch closed-loop eigenvalues
% A_pitch = [0            1;
%           -k2_pitch/Iy -k1_pitch/Iy];
% 
% eig_pitch = eig(A_pitch);
% 
% disp('Pitch eigenvalues:')
% disp(eig_pitch)

%% 3.2

% function [Fc, Gc] = InnerLoopFeedback(var)
% 
% % State order:
% % var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]
% 
% phi   = var(7);
% theta = var(8);
% 
% p = var(10);
% q = var(11);
% r = var(12);
% 
% % Hard-coded parameters
% m = 0.068;
% g = 9.81;
% 
% Ix = 5.8e-5;
% Iy = 7.2e-5;
% 
% % Gains from Problem 3.1
% k1_roll  = 12*Ix;
% k2_roll  = 20*Ix;
% 
% k1_pitch = 12*Iy;
% k2_pitch = 20*Iy;
% 
% k_r = 0.004;
% 
% % Force vector
% Fc = [0; 0; -m*g];
% 
% % Moment vector
% L_c = -k1_roll*p  - k2_roll*phi;
% M_c = -k1_pitch*q - k2_pitch*theta;
% N_c = -k_r*r;
% 
% Gc = [L_c; M_c; N_c];
% 
% end

%% 3.3

% clear;
% clc;
% close all;
% 
% m  = 0.068;
% g  = 9.81;
% Ix = 5.8e-5;
% Iy = 7.2e-5;
% Iz = 1.0e-4;
% I  = diag([Ix Iy Iz]);
% 
% opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
% tspan = [0 10];
% 
% % Choose one case at a time
% var0 = zeros(12,1);
% 
% % a) +5 deg roll
% var0(7) = deg2rad(5);
% 
% % b) +5 deg pitch
% % var0(8) = deg2rad(5);
% 
% % c) +0.1 roll rate
% % var0(10) = 0.1;
% 
% % d) +0.1 pitch rate
% % var0(11) = 0.1;
% 
% odefun = @(t,var) QuadrotorEOM_Linearized_InnerLoop(t,var,g,m,I);
% [t,y] = ode45(odefun,tspan,var0,opts);
% 
% % Build control history
% control_input_array = zeros(length(t),4);
% for i = 1:length(t)
%     [Fc,Gc] = InnerLoopFeedback(y(i,:)');
%     control_input_array(i,:) = [Fc(3), Gc(1), Gc(2), Gc(3)];
% end
% 
% y_plot = [y(:,1:3), rad2deg(y(:,7:9)), y(:,4:6), y(:,10:12)];
% PlotAircraftSim(t, y_plot, control_input_array, [1:6], 'b-');


%% 3.4

% same as 3.3 but with non linear model

% function var_dot = QuadrotorEOM_Nonlinear_InnerLoop(t, var, g, m, I, d, km, nu, mu)
% 
% % State order:
% % var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]
% 
% u = var(4);
% v = var(5);
% w = var(6);
% 
% phi   = var(7);
% theta = var(8);
% psi   = var(9);
% 
% p = var(10);
% q = var(11);
% r = var(12);
% 
% R_B2I = angle2dcm(phi, theta, psi, 'XYZ');
% 
% vel_body = [u; v; w];
% pos_dot = R_B2I * vel_body;
% 
% [Fc, Gc] = InnerLoopFeedback(var);
% motor_forces = ComputeMotorForces(Fc, Gc, d, km);
% 
% f1 = motor_forces(1);
% f2 = motor_forces(2);
% f3 = motor_forces(3);
% f4 = motor_forces(4);
% 
% Va = norm(vel_body);
% if Va == 0
%     F_aero = [0; 0; 0];
% else
%     F_aero = -nu * Va * vel_body;
% end
% 
% Fc_body = -(f1 + f2 + f3 + f4);
% F_body = [0; 0; Fc_body];
% 
% g_inertial = [0; 0; m*g];
% F_gravity_body = R_B2I' * (-g_inertial);
% 
% F_tot_B = F_body + F_gravity_body + F_aero;
% 
% omega_B = [p; q; r];
% vel_dot = (1/m) * F_tot_B - cross(omega_B, vel_body);
% 
% T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
%      0, cos(phi),            -sin(phi);
%      0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
% 
% euler_dot = T * omega_B;
% 
% L = d/sqrt(2) * (-f1 - f2 + f3 + f4);
% M = d/sqrt(2) * ( f1 - f2 - f3 + f4);
% N = km * (f1 - f2 + f3 - f4);
% 
% omega_mag = norm(omega_B);
% if omega_mag == 0
%     M_aero = [0; 0; 0];
% else
%     M_aero = -mu * omega_mag * omega_B;
% end
% 
% moments = [L; M; N] + M_aero;
% 
% omega_B_dot = I \ (moments - cross(omega_B, I*omega_B));
% 
% var_dot = zeros(12,1);
% var_dot(1:3)   = pos_dot;
% var_dot(4:6)   = vel_dot;
% var_dot(7:9)   = euler_dot;
% var_dot(10:12) = omega_B_dot;
% 
% end

%% 3.5

% clear; clc; close all;
% 
% g  = 9.81;
% Ix = 5.8e-5;
% 
% k1 = 12*Ix;
% k2 = 20*Ix;
% 
% k3_values = linspace(0,0.01,500);
% 
% eig_store = zeros(length(k3_values),3);
% 
% for i = 1:length(k3_values)
%     k3 = k3_values(i);
% 
%     A_lat = [0      g      0;
%              0      0      1;
%             -k3/Ix -k2/Ix -k1/Ix];
% 
%     eig_store(i,:) = eig(A_lat).';
% end
% 
% figure
% plot(real(eig_store(:,1)), imag(eig_store(:,1)), 'b.'); hold on
% plot(real(eig_store(:,2)), imag(eig_store(:,2)), 'r.');
% plot(real(eig_store(:,3)), imag(eig_store(:,3)), 'k.');
% grid on
% xlabel('Real')
% ylabel('Imaginary')
% title('Lateral Eigenvalue Locus vs k_3')
% legend('\lambda_1','\lambda_2','\lambda_3')

%% 3.6


% function [Fc, Gc] = VelocityReferenceFeedback(t, var)
% 
% % State order:
% % var = [xE; yE; zE; u; v; w; phi; theta; psi; p; q; r]
% 
% u = var(4);
% v = var(5);
% 
% phi   = var(7);
% theta = var(8);
% 
% p = var(10);
% q = var(11);
% r = var(12);
% 
% % Hard-coded parameters
% m  = 0.068;
% g  = 9.81;
% Ix = 5.8e-5;
% Iy = 7.2e-5;
% 
% % Inner-loop gains
% k1_roll  = 12*Ix;
% k2_roll  = 20*Ix;
% 
% k1_pitch = 12*Iy;
% k2_pitch = 20*Iy;
% 
% k_r = 0.004;
% 
% % Example outer-loop gains
% k3_lat  = 5e-4;
% k3_long = 5e-4;
% 
% % Reference command: 0.5 m/s for first 2 sec, then 0
% if t <= 2
%     u_r = 0.5;
%     v_r = 0.0;
% else
%     u_r = 0.0;
%     v_r = 0.0;
% end
% 
% Fc = [0; 0; -m*g];
% 
% L_c = -k1_roll*p  - k2_roll*phi  + k3_lat*(v_r - v);
% M_c = -k1_pitch*q - k2_pitch*theta + k3_long*(u_r - u);
% N_c = -k_r*r;
% 
% Gc = [L_c; M_c; N_c];
% 
% end

%% 3.7


% function var_dot = QuadrotorEOM_Nonlinear_VelRef(t, var, g, m, I, d, km, nu, mu)
% 
% u = var(4);
% v = var(5);
% w = var(6);
% 
% phi   = var(7);
% theta = var(8);
% psi   = var(9);
% 
% p = var(10);
% q = var(11);
% r = var(12);
% 
% R_B2I = angle2dcm(phi, theta, psi, 'XYZ');
% 
% vel_body = [u; v; w];
% pos_dot = R_B2I * vel_body;
% 
% [Fc, Gc] = VelocityReferenceFeedback(t, var);
% motor_forces = ComputeMotorForces(Fc, Gc, d, km);
% 
% f1 = motor_forces(1);
% f2 = motor_forces(2);
% f3 = motor_forces(3);
% f4 = motor_forces(4);
% 
% Va = norm(vel_body);
% if Va == 0
%     F_aero = [0; 0; 0];
% else
%     F_aero = -nu * Va * vel_body;
% end
% 
% Fc_body = -(f1 + f2 + f3 + f4);
% F_body = [0; 0; Fc_body];
% 
% g_inertial = [0; 0; m*g];
% F_gravity_body = R_B2I' * (-g_inertial);
% 
% F_tot_B = F_body + F_gravity_body + F_aero;
% 
% omega_B = [p; q; r];
% vel_dot = (1/m) * F_tot_B - cross(omega_B, vel_body);
% 
% T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
%      0, cos(phi),            -sin(phi);
%      0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
% 
% euler_dot = T * omega_B;
% 
% L = d/sqrt(2) * (-f1 - f2 + f3 + f4);
% M = d/sqrt(2) * ( f1 - f2 - f3 + f4);
% N = km * (f1 - f2 + f3 - f4);
% 
% omega_mag = norm(omega_B);
% if omega_mag == 0
%     M_aero = [0; 0; 0];
% else
%     M_aero = -mu * omega_mag * omega_B;
% end
% 
% moments = [L; M; N] + M_aero;
% 
% omega_B_dot = I \ (moments - cross(omega_B, I*omega_B));
% 
% var_dot = zeros(12,1);
% var_dot(1:3)   = pos_dot;
% var_dot(4:6)   = vel_dot;
% var_dot(7:9)   = euler_dot;
% var_dot(10:12) = omega_B_dot;
% 
% end