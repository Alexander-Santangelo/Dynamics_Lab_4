
function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

p = var(10);
q = var(11);
r = var(12);

Fc = transpose([0,0,-m*g]);

Gc = transpose([-.004*p, -.004*q, -.004*r]);

end