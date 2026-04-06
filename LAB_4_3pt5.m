%% 3.5

clear; clc; close all;

g  = 9.81;
Ix = 5.8e-5;
Iy = 7.2e-5;

k1_lat = .0073; % CHANGE TO FIT REST OF MODEL
k2_lat = 0.0144; % CHANGE TO FIT REST OF MODEL

k3_lat_values = linspace(0,0.01,500);

eig_store_lat = zeros(length(k3_lat_values),3);

for i = 1:length(k3_lat_values)
    k3 = k3_lat_values(i);

    A_lat = [0      g      0;
             0      0      1;
            -k3/Ix -k2_lat/Ix -k1_lat/Ix];

    eig_store_lat(i,:) = eig(A_lat).';
end

%% --- Plot FULL eigenvalue locus (complex plane) ---
figure
hold on
for i = 1:3
    plot(real(eig_store_lat(:,i)), imag(eig_store_lat(:,i)), '.')
end
grid on
xlabel('Real')
ylabel('Imaginary')
title('Lateral Eigenvalue Locus vs k_3')

%% --- Filter: real eigenvalues only ---
rows_real_lat = all(abs(imag(eig_store_lat)) < 1e-6, 2);
lat_real = eig_store_lat(rows_real_lat, :);
k3_real_lat = k3_lat_values(rows_real_lat);

%% --- Time constant condition: lambda <= -0.8 ---
rows_fast_lat = all(real(lat_real) <= -0.8, 2);
lat_valid = lat_real(rows_fast_lat, :);
k3_valid_lat = k3_real_lat(rows_fast_lat);

disp('Valid k3 (lateral):')
disp(k3_valid_lat')


%% ================= LONGITUDINAL =================

k1_long = .0059; % CHANGE TO FIT REST OF MODEL
k2_long = 0.0116; % CHANGE TO FIT REST OF MODEL

% NOTE: try NEGATIVE k3 for longitudinal (sign matters!)
k3_long_values = linspace(-0.01,0,500);

eig_store_long = zeros(length(k3_long_values),3);

for i = 1:length(k3_long_values)
    k3 = k3_long_values(i);

    A_long = [0      -g      0;
              0       0      1;
             -k3/Iy -k2_long/Iy -k1_long/Iy];

    eig_store_long(i,:) = eig(A_long).';
end

%% Plot locus
figure
hold on
for i = 1:3
    plot(real(eig_store_long(:,i)), imag(eig_store_long(:,i)), '.')
end
grid on
xlabel('Real')
ylabel('Imaginary')
title('Longitudinal Eigenvalue Locus vs k_3')

%% Filter real eigenvalues
rows_real_long = all(abs(imag(eig_store_long)) < 1e-6, 2);
long_real = eig_store_long(rows_real_long, :);
k3_real_long = k3_long_values(rows_real_long);

%% Time constant condition
rows_fast_long = all(real(long_real) <= -0.8, 2);
long_valid = long_real(rows_fast_long, :);
k3_valid_long = k3_real_long(rows_fast_long);

disp('Valid k3 (longitudinal):')
disp(k3_valid_long')
