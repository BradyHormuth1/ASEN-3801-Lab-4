function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

x = var(1,:);
p = var(10,:);
q = var(11,:);
r = var(12,:);


Fc = zeros(length(x),3);
Gc = zeros(length(x),3);


for i = 0:length(x)
    Fc(i,3) = m*g;
    Gc(i,1) = -0.004*p(i);
    Gc(i,2) = -0.004*q(i);
    Gc(i,3) = -0.004*r(i);
end
end

