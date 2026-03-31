function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

% vlaues 


% time = time;
% aircraft_state_array = aircraft_state_array;
% control_input_array = control_input_array;
% fig = fig;
% col = col;



n = length(time);

%% inertial position, 
figure(fig(1));

subplot(3,1,1);
plot(time, aircraft_state_array(:,1), col,LineWidth=2); hold on; grid on;
ylabel('x_E (m)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');
title('Inertial Position vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,2), col,LineWidth=2); hold on; grid on;
ylabel('y_E (m)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(3,1,3);
plot(time, aircraft_state_array(:,3).* -1, col,LineWidth=2); hold on; grid on;
ylabel('z_E (m)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

print('5degRollRate1','-dpng','-r300');


%% Euler angles, 
figure(fig(2));

subplot(3,1,1);
plot(time, rad2deg(aircraft_state_array(:,4)), col,LineWidth=2); hold on; grid on;
ylabel('\phi (deg)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

title('Euler angles vs Time');

subplot(3,1,2);
plot(time, rad2deg(aircraft_state_array(:,5)), col,LineWidth=2); hold on; grid on;
ylabel('\theta (deg)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(3,1,3);
plot(time, rad2deg(aircraft_state_array(:,6)), col,LineWidth=2); hold on; grid on;
ylabel('\psi (deg)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

print('5degRollRate2','-dpng','-r300');

%% inertial velocity in body frame
figure(fig(3));

subplot(3,1,1);
plot(time, aircraft_state_array(:,7), col,LineWidth=2); hold on; grid on;
ylabel('u (m/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

title('inertial velocity in body frame vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,8), col,LineWidth=2); hold on; grid on;
ylabel('v (m/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(3,1,3);
plot(time, aircraft_state_array(:,9), col,LineWidth=2); hold on; grid on;
ylabel('w (m/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

print('5degRollRate3','-dpng','-r300');

%% angular velocity
figure(fig(4));

subplot(3,1,1);
plot(time, rad2deg(aircraft_state_array(:,10)), col,LineWidth=2); hold on; grid on;
ylabel('p (deg/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

title('angular velocity vs Time');

subplot(3,1,2);
plot(time, rad2deg(aircraft_state_array(:,11)), col,LineWidth=2); hold on; grid on;
ylabel('q (deg/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(3,1,3);
plot(time, rad2deg(aircraft_state_array(:,12)), col,LineWidth=2); hold on; grid on;
ylabel('r (deg/s)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

print('5degRollRate4','-dpng','-r300');

% % control_input_array
figure(fig(5));

subplot(4,1,1);
plot(time, control_input_array(1,:), col,LineWidth=2); hold on; grid on;
ylabel('Zc (Nm)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(4,1,2);
plot(time, control_input_array(2,:), col,LineWidth=2); hold on; grid on;
ylabel('Lc (Nm)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(4,1,3);
plot(time, control_input_array(3,:), col,LineWidth=2); hold on; grid on;
ylabel('Mc (Nm)');
xlabel('Time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

subplot(4,1,4);
plot(time, control_input_array(4,:), col,LineWidth=2); hold on; grid on;
ylabel('Nc (Nm)');
xlabel('time (s)');
legend('Nonlinear EOMs','Linear EOMs',Location='southwest');

title(' control_input_array ');


print('5degRollRate5','-dpng','-r300');

%% 3D path
figure(fig(6));
plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),aircraft_state_array(:,3).*-1,LineWidth=2); hold on
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),aircraft_state_array(1,3).*-1, 'g*',LineWidth=2); hold on
plot3(aircraft_state_array(n,1),aircraft_state_array(n,2),aircraft_state_array(n,3).*-1, 'r*',LineWidth=2); hold on
grid on;
title(' 3D path ');
xlabel('(m)');
ylabel('(m)');
zlabel('(m)');
print('5degRollRate6','-dpng','-r300');

end