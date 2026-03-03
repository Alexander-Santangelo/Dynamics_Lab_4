clear; clc; close all

%% load data
load('RSdata_nocontrol.mat')



%% values 
Mass = 0.068; % kg
d = 0.060;    % m
km = 0.0024; % Nm/N

Ix = 5.8e-5; % kg*m^2
Iy = 7.2e-5; % kg*m^2
Iz = 1.0e-4; % kg*m^2

v_v = 1e-3; % N/(m/s)^2
Mu = 2e-6; % N*M/(rad/s)^2


%% Fucntion
aircraft_state_array = rt_estim.signals.values(:,:);
time = rt_estim.time(:);



%% definitons
x_E = aircraft_state_array(1,:);
y_E = aircraft_state_array(2,:);
z_E = aircraft_state_array(3,:);

psi = aircraft_state_array(4,:);
theta = aircraft_state_array(5,:);
phi = aircraft_state_array(6,:);

u = aircraft_state_array(7,:);
v = aircraft_state_array(8,:);
w = aircraft_state_array(9,:);

p = aircraft_state_array(10,:);
q = aircraft_state_array(11,:);
r = aircraft_state_array(12,:);




% 
% x_E = rt_estim.signals.values(1,:);
% y_E = rt_estim(2,:);
% z_E = rt_estim(3,:);
% 
% psi = rt_estim(4,:);
% theta = rt_estim(5,:);
% phi = rt_estim(6,:);
% 
% u = rt_estim(7,:);
% v = rt_estim(8,:);
% w = rt_estim(9,:);
% 
% p = rt_estim(10,:);
% q = rt_estim(11,:);
% r = rt_estim(12,:);









PlotAircraftSim(time, aircraft_state_array, 0 , [1:6] , 'b-');



