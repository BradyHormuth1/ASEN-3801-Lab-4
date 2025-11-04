function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% RotationDerivativeFeedback: Calculates the control vectors Fc and Gc.
%
% Inputs: 
%       var: 12x1 aircraft state vector
%       m: mass of quadcopter
%       g: acceleration due to gravity
%
% Outputs:
%       Fc: Control Force Vector
%       Gc: Control Moment Vector


x = var(1);
p = var(10);
q = var(11);
r = var(12);

Fc = [0;0;0];
Gc = [0;0;0];

% Control force proportional to weight
Fc(3) = m*g;

% Control moments about each body axis proportional to the rotational rates 
% about their respective axes, but in the opposite sign of the angular 
% velocity with a gain of 0.004 Nm/(rad/sec)
Gc(1) = -0.004*p;
Gc(2) = -0.004*q;
Gc(3) = -0.004*r;

end

