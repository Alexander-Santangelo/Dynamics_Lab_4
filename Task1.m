clear; 
clc; 
close all; 

% Defining Givens

g = 9.81; % m/s^2
m = 0.068; % kg
d = 0.06; % m
km = 0.0024; % Nm/N
I = [5.8*10^(-5),0,0;0,7.2*10^(-5),0;0,0,1.0*10^(-4)]; % kg*m^2
nu = 1*10^(-3); % N/(m/s)^2
mu = 2*10^(-6); % Nm / (rad/2)^2
motor_forces = [(g/4)*m;(g/4)*m;(g/4)*m;(g/4)*m]; % N

% % Reading in Data
% load('RSdata_nocontrol.mat'); 
% times = rt_estim.time(:);
% xdata = rt_estim.signals.values(:,1);
% var0 = [rt_estim.signals.values(1,1),rt_estim.signals.values(2,1),rt_estim.signals.values(3,1),...
%     rt_estim.signals.values(4,1),rt_estim.signals.values(5,1),rt_estim.signals.values(6,1),rt_estim.signals.values(7,1),...
%     rt_estim.signals.values(8,1),rt_estim.signals.values(9,1),...
%     rt_estim.signals.values(10,1),rt_estim.signals.values(11,1),rt_estim.signals.values(12,1)];

% Calling ODE45

var0 = zeros(1,12);
t = [0 10];
[t,var] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces),t,var0);

% Calling plotting function
fig = 1:6; 
PlotAircraftSim(t, var, motor_forces, fig', "b-");

