function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)

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

% Decomposing Control Moments 
Lc = deltaGc(1); 
Mc = deltaGc(2); 
Nc = deltaGc(3); 

% Linearized Model

pdotE = [uE;vE;wE]; 
o_vector = [p;q;r];
vdotEB = g*[-theta;phi;0] + (1/m) * [0;0;deltaFc];
omegadotEB = [(1/Ix)*Lc;(1/Iy)*Mc;(1/Iz)*Nc];

var_dot = [pdotE;o_vector;vdotEB;omegadotEB];

end