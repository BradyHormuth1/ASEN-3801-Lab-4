
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

% PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% time:               [n×1]
% aircraft_state_array: [n×12] ordered as:
%   [x y z phi theta psi uE vE wE p q r]
% control_input_array: [4×n] rows = [Zc; Lc; Mc; Nc]
% fig:                 vector of figure numbers (len ≥ 6)
% col:                 line style/color for 2nd traces (e.g., 'b-')

% --------- validate/shape ----------
time = time(:);
X = aircraft_state_array;
if size(X,1) ~= numel(time) && size(X,2) == numel(time)
    X = X.'; % make rows = time
end
assert(size(X,1) == numel(time) && size(X,2) == 12, ...
    'aircraft_state_array must be [n×12] aligned with time');

% Unpack the rt_etim states
XE = X(:,1); 
YE = X(:,2); 
ZE = X(:,3);
phi = X(:,4); 
theta = X(:,5); 
psi = X(:,6);
uE = X(:,7); 
vE = X(:,8); 
wE = X(:,9);
p = X(:,10); 
q = X(:,11); 
r = X(:,12);

tol = 1e-14;
XE(abs(XE) < tol) = 0;

% Unpack controls from control_input_array
Zc = control_input_array(1,:); 
Lc = control_input_array(2,:);
Mc = control_input_array(3,:);
Nc = control_input_array(4,:);

%---------line styles ------------------

if nargin < 5 || isempty(col)
    col = 'r-';
end

% primary call (solid) vs secondary call (dotted)
isSecondary = contains(col,'--') || contains(col,':');

if isSecondary
    redStyle   = col;   % whatever you passed, e.g. 'b--' or 'r:'
    greenStyle = 'g--';  % dotted green
    blueStyle  = 'b--';  % dotted blue
    whiteStyle = 'w--'; % for 3D and Nc
else
    redStyle   = col;   % e.g. 'r-'
    greenStyle = 'g-';
    blueStyle  = 'b-';
    whiteStyle = 'w-';
end
lw = 1.25; %linewidth

% ---------------- PLOTTING ----------------

% Fig 1: Inertial Position
figure(fig(1));

subplot(3,1,1); 
plot(time,XE, redStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('X_E [m]');
axis tight;

subplot(3,1,2); 
plot(time,YE,greenStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('Y_E [m]');
axis tight;

subplot(3,1,3); 
plot(time,ZE, blueStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('Z_E [m]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Inertial Position');


% Fig 2: Euler Angles
figure(fig(2));

subplot(3,1,1); 
plot(time,phi, redStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('\phi [rad]');
axis tight;

subplot(3,1,2); 
plot(time,theta,greenStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('\theta [rad]');
axis tight;

subplot(3,1,3); 
plot(time,psi,blueStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('\psi [rad]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Euler Angles');


% Fig 3: Inertial Velocity
figure(fig(3));

subplot(3,1,1); 
plot(time,uE,redStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('u_E [m/s]');
axis tight;

subplot(3,1,2); 
plot(time,vE,greenStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('v_E [m/s]');
axis tight;

subplot(3,1,3); 
plot(time,wE,blueStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('w_E [m/s]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Inertial Velocity');

% Fig 4: Angular Velocity
figure(fig(4));

subplot(3,1,1); 
plot(time,p,redStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('p [rad/s]');
axis tight;

subplot(3,1,2); 
plot(time,q,greenStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('q [rad/s]');
axis tight;

subplot(3,1,3); 
plot(time,r,blueStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('r [rad/s]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Angular Velocity');


% Fig 5: Control Inputs
figure(fig(5)); 
tiledlayout(4,1);

nexttile; 
plot(time,Zc,redStyle,'LineWidth',lw); 
grid on;
hold on; 
ylabel('Z_c');
axis tight;

nexttile; 
plot(time,Lc,greenStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('L_c');
axis tight;

nexttile; 
plot(time,Mc,blueStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('M_c');
axis tight;

nexttile; 
plot(time,Nc,whiteStyle,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('N_c'); 
axis tight;

xlabel('Time [s]');
sgtitle('Control Inputs');

% Fig 6: 3D Trajectory
figure(fig(6)); 
hold on; 
grid on; 
Zup = -ZE; %  NED convention
plot3(XE, YE, Zup, whiteStyle, 'LineWidth', lw);
plot3(XE(1), YE(1), Zup(1), 'go', 'MarkerSize', 7);
plot3(XE(end), YE(end), Zup(end), 'rx', 'MarkerSize', 7);
xlabel('X_E [m]'); 
ylabel('Y_E [m]'); 
zlabel('Z_{up} [m]');
title('3D Trajectory: Start=Green, End=Red');
view(45,45);

end
