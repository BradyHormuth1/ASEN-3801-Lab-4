
%% Parameters ------------------------------------------------------------
m  = 0.068;
d  = 0.060;
km = 0.0024;
Ix = 5.8e-5; 
Iy = 7.2e-5; 
Iz = 1.0e-4; 
I = [Ix; Iy; Iz];
g  = 9.81;

nu = 1.0e-3;
mu = 2.0e-6;

% -------------------------------------------------------------------------
%% 1.3 HOVER TRIM SIM
% -------------------------------------------------------------------------
var0 = [0 0 0 ...      % x y z
        0 0 0 ...      % phi theta psi
        0 0 0 ...      % uE vE wE
        0 0 0].';      % p q r

% hover rotor forces
f_hover = m*g/4;
motor_trim = [f_hover; f_hover; f_hover; f_hover];

% call ode to simulate
tspan = [0 10];
ode   = @(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_trim);
opts  = odeset('RelTol',1e-4,'AbsTol',1e-6);
[t, Y] = ode45(ode, tspan, var0, opts);

% controls from QuadEOM
[~, Zc_h, Lc_h, Mc_h, Nc_h] = QuadrotorEOM(0, var0, g, m, I, d, km, nu, mu, motor_trim);
control_hover = [Zc_h; Lc_h; Mc_h; Nc_h] * ones(1, numel(t));   % 4 x N

% plot
fig = 1:6;
col = 'r-';
PlotAircraftSim(t, Y, control_hover, fig, col);

% output check for thrust
fprintf('Hover: Zc trim = %.6f N   (total upward thrust = %.6f N)\n\n', Zc_h, -Zc_h);

% -------------------------------------------------------------------------
%% 1.4a simulate 5 m/s no yaw
% -------------------------------------------------------------------------

% desired inertial velocity
Ux = 0; 
Uy = 5; % 5m/s East (will be change in phi)
Uz = 0;

phi_5 = asin(nu * Uy^2 / (m*g)); % phi as derived for conditions
theta_5 = 0;
psi_5 = 0; % no yaw

% rolled thrust
Zc = -m * g * cos(phi_5);

% define the forces 
f_5 = - Zc / 4;
motor_trim_5 = [f_5; f_5; f_5; f_5];

% define the variable vec
var0_5 = [0 0 0 ...                   % x y z
              phi_5 theta_5 psi_5 ...  % phi theta psi
              0 Uy 0 ...                  % uE vE wE  (0, 5, 0)
              0 0 0].';                   % p q r

%call ode to simulate
ode_5 = @(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_trim_5);
[t2, Y2] = ode45(ode_5, tspan, var0_5, opts);

% get controls from QuadEOM
[~, Zc_5, Lc_5, Mc_5, Nc_5] = QuadrotorEOM(0, var0_5, g, m, I, d, km, nu, mu, motor_trim_5);
controls_5 = [Zc_5; Lc_5; Mc_5; Nc_5] * ones(1, numel(t2));

% plot 
fig2 = 7:12;
PlotAircraftSim(t2, Y2, controls_5, fig2, col);

fprintf('No Yaw 5 m/s EAST: phi_trim = %.6f rad (%.3f deg)\n', phi_5, rad2deg(phi_5));
fprintf('No Yaw 5 m/s EAST: Zc_trim  = %.6f N   (total thrust = %.6f N)\n', Zc_5, -Zc_5);
fprintf('No Yaw 5 m/s EAST: rotor trim = %.6f N each\n\n', f_5);

% -------------------------------------------------------------------------
%% 1.4b simulate 5 m/s 90 deg yaw
% -------------------------------------------------------------------------

psi_yaw = pi/2; % no yaw

% define the variable vec
var0_5 = [0 0 0 ...                   % x y z
              phi_5 theta_5 psi_yaw ...  % phi theta psi(90)
              0 Uy 0 ...                  % uE vE wE  (0, 5, 0)
              0 0 0].';                   % p q r

%call ode to simulate
ode_yaw = @(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_trim_5);
[t3, Y3] = ode45(ode_yaw, tspan, var0_5, opts);

% get controls from QuadEOM
[~, Zc_yaw, Lc_yaw, Mc_yaw, Nc_yaw] = QuadrotorEOM(0, var0_5, g, m, I, d, km, nu, mu, motor_trim_5);
controls_yaw = [Zc_yaw; Lc_yaw; Mc_yaw; Nc_yaw] * ones(1, numel(t2));

% plot 
fig3 = 13:18;
PlotAircraftSim(t3, Y3, controls_yaw, fig3, col);

fprintf('Yaw 5 m/s EAST: phi_trim = %.6f rad (%.3f deg)\n', phi_5, rad2deg(phi_5));
fprintf('Yaw 5 m/s EAST: Zc_trim  = %.6f N   (total thrust = %.6f N)\n', Zc_yaw, -Zc_yaw);
fprintf('Yaw 5 m/s EAST: rotor trim = %.6f N each\n\n', f_5);

% -------------------------------------------------------------------------
%% 1.5 No control Data
% -------------------------------------------------------------------------

% load
noControl = load('RSdata_nocontrol.mat');

% extract time and state
time = noControl.rt_estim.time(:); %nx1
state0 = noControl.rt_estim.signals.values;

% swap phi and psi for preference (4 +6)
state1 = state0(:, [1 2 3 6 5 4 7 8 9 10 11 12]);

motor_data = noControl.rt_motor.signals.values;
motor_d = motor_data.'; %nx4 to 4xn

fig4 = 19:24;
PlotAircraftSim(time, state1, motor_d, fig4, col);


% Print summary for stability analysis
fprintf('\nHover Stability Test (No Control)\n');
fprintf('Duration: %.2f s\n', time(end) - time(1));
fprintf('Initial position: [%.3f %.3f %.3f] m\n', state1(1,1), state1(1,2), state1(1,3));
fprintf('Final position:   [%.3f %.3f %.3f] m\n', state1(end,1), state1(end,2), state1(end,3));
fprintf('Change in Z: %.3f m\n', abs(state1(end,3) - state1(1,3)));

% -------------------------------------------------------------------------
