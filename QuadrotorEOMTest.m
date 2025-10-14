clear; clc; close all;

m = 0.068; %kg
d = 0.060; %m
km = 0.0024; %N*m/(N)
Ix = 5.8e-5; %kg*m^2
Iy = 7.2e-5; %kg*m^2
Iz = 1.0e-4; %kg*m^2
Imatrix = [Ix; Iy; Iz];
g = 9.81; %m/s^2

var_2 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

tspan = 0:1:10; %s

W = m*g;
Zc_2 = -W;
motor_forces_2 = [-Zc_2/4; -Zc_2/4; -Zc_2/4; -Zc_2/4];
control_2 = [Zc_2; 0; 0; 0];
Gc = [0; 0; 0];

[t, var_dot] = ode45(@(tspan, var_2) QuadrotorEOM(tspan, var_2, g, m, Imatrix, d, km, 0, 0, motor_forces_2), tspan, var_2);


load("RSdata_nocontrol.mat");
time = rt_estim.time(:);
PlotAircraftSim(rt_estim, var_2, control_2, [1 2 3 4 5 6], 'b-');




