function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
%% Calculating EOMs from Givens

% Decomposing State Vector 
xE = var(1); 
yE = var(2); 
zE = var(3); 

phi = var(4); 
theta = var(5); 
psi = var(6); 

uE = var(7); 
vE = var(8); 
wE = var(9); 

p = var(10); 
q = var(11); 
r = var(12); 

% Decomposing I
Ix = I(1,1);  
Iy = I(2,2); 
Iz = I(3,3); 

% Defining R Vectors 
R_e_to_b = [cosd(theta)*cosd(psi),cosd(theta)*sind(psi),-sind(theta);
            sind(phi)*sind(theta)*cosd(psi) - cosd(phi)*sind(psi),sind(phi)*sind(theta)*sind(psi) + cosd(phi)*cosd(psi),sind(phi)*cosd(theta);
            cosd(phi)*sind(theta)*cosd(psi) + sind(phi)*sind(psi), cosd(phi)*sind(theta)*sind(psi) - sind(phi)*cosd(psi),cosd(phi)*cosd(theta)];

% Calculating Zc, Lc, Mc, and Nc

cVec = [-1,-1,-1,-1;-d/sqrt(2),-d/sqrt(2),d/sqrt(2),d/sqrt(2);d/sqrt(2),-d/sqrt(2),-d/sqrt(2),d/sqrt(2);km,-km,km,-km] * motor_forces;
Zc = cVec(1); 
Lc = cVec(2); 
Mc = cVec(3); 
Nc = cVec(4); 

% Calculating X, Y, and Z
Va = sqrt(uE^2+vE^2+wE^2); 
posVec = -nu * Va * [uE;vE;wE];
X = posVec(1); 
Y = posVec(2); 
Z = posVec(3); 

% Calculating L, M and N
momVec = -mu * sqrt(p^2+q^2+r^2) * [p;q;r]; 
L = momVec(1); 
M = momVec(2);
N = momVec(3); 

% Block 1 (Translational Kinematics)

pdotE = R_e_to_b' * [uE;vE;wE]; 

% Block 2 (Rotational Kinematics)

o_vector = [1,sind(phi)*tand(theta),cosd(phi)*tand(theta);
            0,cosd(phi),-sind(phi);
            0,sind(phi)*secd(theta),cosd(phi)*secd(theta)] * [p;q;r];

% Block 3 (Translational Dynamics)

vdotEB = [r*vE-q*wE;p*wE-r*uE;p*uE-p*vE] + g*[-sind(theta);cosd(theta)*sind(phi);cosd(theta)*cosd(phi)] + (1/m)*[X;Y;Z+Zc]; 

% Block 4 (Roational Dynamics)

omegadotEB = [((Iy-Iz)/Ix)*q*r;((Iz-Ix)/Iy)*p*r;((Ix-Iy)/Iz)*p*q] + [(1/Ix)*L;(1/Iy)*M;(1/Iz)*N] + [(1/Ix)*Lc;(1/Iy)*Mc;(1/Iz)*Nc];

% Combining results 

var_dot = [pdotE;o_vector;vdotEB;omegadotEB];

end