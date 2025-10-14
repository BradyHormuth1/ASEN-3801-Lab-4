
%----------------- TEMP MAIN FOR TESTING-------------------
% clean command window but do not clear workspace
clc;

% defin input control array as command signal values
control_input_array = rt_cmd.signals.values';

%call the function
PlotAircraftSim(rt_estim, control_input_array, [1 2 3 4 5 6], 'b-');


% ------------- FUNCTION --------------------
% Plot Quadrotor function
function PlotAircraftSim(rt_estim, control_input_array, fig, col)

% ---------------- EXTRACT DATA ----------------
time = rt_estim.time(:);  %time column
X = rt_estim.signals.values;   % 12×n state array

if size(X,2) ~= numel(time)
    X = X.';  % ensure 12×n orientation
end

% Unpack the rt_etim states
XE = X(1,:); 
YE = X(2,:); 
ZE = X(3,:);
psi = X(4,:); 
theta = X(5,:); 
phi = X(6,:);
uE = X(7,:); 
vE = X(8,:); 
wE = X(9,:);
p = X(10,:); 
q = X(11,:); 
r = X(12,:);

% Unpack controls from control_input_array
Zc = control_input_array(1,:); 
Lc = control_input_array(2,:);
Mc = control_input_array(3,:);
Nc = control_input_array(4,:);


% ---------------- PLOTTING ----------------
lw = 1.25; %linewidth
% Fig 1: Inertial Position
figure(fig(1));

subplot(3,1,1); 
plot(time,XE, 'r','LineWidth',lw); 
grid on; 
hold on; 
ylabel('X_E [m]');
axis tight;

subplot(3,1,2); 
plot(time,YE,col,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('Y_E [m]');
axis tight;

subplot(3,1,3); 
plot(time,ZE,'g','LineWidth',lw); 
grid on; 
hold on; 
ylabel('Z_E [m]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Inertial Position');


% Fig 2: Euler Angles
figure(fig(2));

subplot(3,1,1); 
plot(time,psi,'r','LineWidth',lw); 
grid on; 
hold on; 
ylabel('\psi [rad]');
axis tight;

subplot(3,1,2); plot(time,theta,col,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('\theta [rad]');
axis tight;

subplot(3,1,3); 
plot(time,phi,'g','LineWidth',lw); 
grid on; 
hold on; 
ylabel('\phi [rad]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Euler Angles');


% Fig 3: Inertial Velocity
figure(fig(3));

subplot(3,1,1); 
plot(time,uE,'r','LineWidth',lw); 
grid on; 
hold on; 
ylabel('u_E [m/s]');
axis tight;

subplot(3,1,2); 
plot(time,vE,col,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('v_E [m/s]');
axis tight;

subplot(3,1,3); 
plot(time,wE,'g','LineWidth',lw); 
grid on; 
hold on; 
ylabel('w_E [m/s]'); 
axis tight;

xlabel('Time [s]');
sgtitle('Inertial Velocity');

% Fig 4: Angular Velocity
figure(fig(4));

subplot(3,1,1); 
plot(time,p,'r','LineWidth',lw); 
grid on; 
hold on; 
ylabel('p [rad/s]');
axis tight;

subplot(3,1,2); 
plot(time,q,col,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('q [rad/s]');
axis tight;

subplot(3,1,3); 
plot(time,r,'g','LineWidth',lw); 
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
plot(time,Zc,'r','LineWidth',lw); 
grid on;
hold on; 
ylabel('Z_c');
axis tight;

nexttile; 
plot(time,Lc,col,'LineWidth',lw); 
grid on; 
hold on; 
ylabel('L_c');
axis tight;

nexttile; 
plot(time,Mc,'g','LineWidth',lw); 
grid on; 
hold on; 
ylabel('M_c');
axis tight;

nexttile; 
plot(time,Nc,'w','LineWidth',lw); 
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
axis equal
Zup = -ZE; % flip sign if NED convention
plot3(XE, YE, Zup, 'w', 'LineWidth', lw);
plot3(XE(1), YE(1), Zup(1), 'go', 'MarkerSize', 7);
plot3(XE(end), YE(end), Zup(end), 'rx', 'MarkerSize', 7);
xlabel('X_E [m]'); 
ylabel('Y_E [m]'); 
zlabel('Z_{up} [m]');
title('3D Trajectory: Start=Green, End=Red');
view(3);

end
