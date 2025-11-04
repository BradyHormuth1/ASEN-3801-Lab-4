function [Fc, Gc] = InnerLoopFeedback(var)
% InnerLoopFeedback: Computes control force and moments for a quadrotor
%
% Inputs:
%   var - 12x1 state vector of the quadrotor:
%         [x; y; z; u; v; w; phi; theta; psi; p; q; r]
%
% Outputs:
%   Fc - control force in body z-direction (scalar)
%   Gc - 3x1 control moments [L; M; N]

m = 1.5;          % mass [kg]
g = 9.81;         % gravity [m/s^2]

Ix = 5.8e-5;      % Roll MOI [kg·m^2]
Iy = 7.2e-5;      % Pitch MOI [kg·m^2]

phi   = var(7);    % roll angle
theta = var(8);    % pitch angle
p     = var(10);   % roll rate
q     = var(11);   % pitch rate
r     = var(12);   % yaw rate

% Roll (phi, p)
Kp_phi = 0.001276;   % proportional gain on roll angle
Kd_phi = 0.00232;   % derivative gain on roll rate

% Pitch (theta, q)
Kp_theta = 0.001584; % proportional gain on pitch angle
Kd_theta = 0.00288; % derivative gain on pitch rate

% Yaw rate (spin) feedback - from Problem 2.3
Kr_r = 0.004;          

Fc = m * g;  % hover condition
L = -Kp_phi*phi - Kd_phi*p;       % roll moment
M = -Kp_theta*theta - Kd_theta*q; % pitch moment
N = -Kr_r*r;                       % yaw moment

Gc = [L; M; N];

end
