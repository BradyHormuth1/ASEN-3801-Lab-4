function [var_dot, Zc, Lc, Mc, Nc] = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% var = [x y z phi theta psi uE vE wE p q r]^T
% I   = [Ix Iy Iz]^T
% motor_forces = [F1 F2 F3 F4]^T  (N)
% Returns var_dot with the same ordering.

% angles, translational velocities, body rates
phi   = var(4);    theta = var(5);   psi  = var(6);
uE    = var(7);    vE    = var(8);   wE   = var(9);
p = var(10); q = var(11); r = var(12);

v_inertial = [uE; vE; wE];

omega_b = [p; q; r];

Ix = I(1); Iy = I(2); Iz = I(3);

% Control allocation (Zc thrust along +z_body (down in NED), moments L,M,N)
controlMomMatrix = [-1,          -1,           -1,          -1;         % Zc
                    -d/sqrt(2),  -d/sqrt(2),    d/sqrt(2),   d/sqrt(2); % Lc
                     d/sqrt(2),  -d/sqrt(2),   -d/sqrt(2),   d/sqrt(2); % Mc
                     km,         -km,           km,         -km];       % Nc

ControlMom = controlMomMatrix * motor_forces;
Zc =  ControlMom(1);  Lc = ControlMom(2);  
Mc = ControlMom(3);  Nc = ControlMom(4);

% snap numerical noise in controls to zero
tol = 1e-14;
if abs(Lc) < tol, Lc = 0; end
if abs(Mc) < tol, Mc = 0; end
if abs(Nc) < tol, Nc = 0; end


% Trig
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);
ttheta  = tan(theta);  sctheta = sec(theta);

% Direction cosine matrix (3-2-1 body->inertial)
R321 = [ cphi*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
         cphi*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
           -sth,                  sphi*cth,                 cphi*cth ];

% Euler-rate mapping
ER = [1,      sphi*ttheta,  cphi*ttheta;
      0,      cphi,         -sphi;
      0,      sphi*sctheta,  cphi*sctheta];

% --- Aerodynamic forces & moments (per-axis quadratic damping) -----------
% Quadratic damping ~ -coeff * component * |component|
F_aero = -nu .* (v_inertial .* norm(v_inertial));    % [N]
M_aero = -mu .* (omega_b    .* norm(omega_b));       % [N·m]

% If you want linear damping instead, swap the two lines above for:
% F_aero = -nu .* v_inertial;   M_aero = -mu .* omega_b;

% Kinematics
inertialPos_dot = R321 * v_inertial;  % [xdot ydot zdot]^T
eulerAng_dot    = ER * omega_b;       % [phidot thetadot psidot]^T

% Translational dynamics (gravity resolved in inertial frame)
uE_dot =  r*vE - q*wE - g*sth          + F_aero(1)/m;
vE_dot =  p*wE - r*uE + g*cth*sphi     + F_aero(2)/m;
wE_dot =  q*uE - p*vE + g*cth*cphi + Zc/m + F_aero(3)/m;
inertialVel_dot = [uE_dot; vE_dot; wE_dot];
Zc = -Zc;
% Rotational dynamics
p_dot = ((Iy - Iz)/Ix)*q*r + Lc/Ix + M_aero(1)/Ix;
q_dot = ((Iz - Ix)/Iy)*p*r + Mc/Iy + M_aero(2)/Iy;
r_dot = ((Ix - Iy)/Iz)*p*q + Nc/Iz + M_aero(3)/Iz;
eulerRate_dot = [p_dot; q_dot; r_dot];

% Pack
var_dot = [inertialPos_dot;
           eulerAng_dot;
           inertialVel_dot;
           eulerRate_dot];
end
