function [Fc, Gc] = VelocityReferenceFeedback(t,var)
    phi = var(4); 
    theta = var(5);
    p = var(10); 
    q = var(11); 
    r = var(12);
    u = var(7); 
    v = var(8); 
    k3 = -5.8116e-04;

    if t < 2
        vr = 0.5; 
        ur = 0.5;
    else 
        vr = 0;
        ur = 0; 
    end 

    Mc = -(0.000864 * q + 0.00144 * theta + k3*(u-ur));
    Lc = -(0.000696 * p + 0.00116 * phi + k3*(v-vr));
    Nc = -0.004*r;
    Zc = -0.068*9.81;

    Fc = [0; 0; Zc];
    Gc = [Lc; Mc; Nc];
    
end