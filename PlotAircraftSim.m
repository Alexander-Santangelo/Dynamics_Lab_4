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
plot(time, aircraft_state_array(:,1), col); hold on;
ylabel('x_E (m)');
xlabel('Time (s)');

title('Inertial Position vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,2), col); hold on;
ylabel('y_E (m)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,3).* -1, col); hold on;
ylabel('z_E (m)');
xlabel('Time (s)');




%% Euler angles, 
figure(fig(2));



subplot(3,1,1);
plot(time, aircraft_state_array(:,4), col); hold on;
ylabel('\phi (deg)');
xlabel('Time (s)');

title('Euler angles vs Time');

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


title('inertial velocity in body frame vs Time');

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
ylabel('p (deg/s)');
xlabel('Time (s)');

title('angular velocity vs Time');

subplot(3,1,2);
plot(time, aircraft_state_array(:,11), col); hold on;
ylabel('q (deg/s)');
xlabel('Time (s)');

subplot(3,1,3);
plot(time, aircraft_state_array(:,12), col); hold on;
ylabel('r (deg/s)');
xlabel('Time (s)');





%% % control_input_array
% figure(fig(5));
% 
% subplot(4,1,1);
% plot(time, control_input_array(1,:), col); hold on;
% ylabel('Zc (Nm)');
% xlabel('Time (s)');
% 
% subplot(4,1,2);
% plot(time, control_input_array(2,:), col); hold on;
% ylabel('Lc (Nm)');
% xlabel('Time (s)');
% 
% subplot(4,1,3);
% plot(time, control_input_array(3,:), col); hold on;
% ylabel('Mc (Nm)');
% xlabel('Time (s)');
% 
% subplot(4,1,4);
% plot(time, control_input_array(4,:), col); hold on;
% ylabel('Nc (Nm)');
% xlabel('time (s)');
% 
% title(' control_input_array ');




%% 3D path
figure(fig(6));



plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),aircraft_state_array(:,3).*-1); hold on
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),aircraft_state_array(1,3).*-1, 'g*'); hold on
plot3(aircraft_state_array(n,1),aircraft_state_array(n,2),aircraft_state_array(n,3).*-1, 'r*'); hold on

title(' 3D path ');
xlabel('(m)');
ylabel('(m)');
zlabel('(m)');




end