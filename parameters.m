%% PARAMETERS
%
% Emiel Legrand
% 01/03/2020
%
% -------------------------------------------------------------------------

param = struct();

%% Quadcopter properties
param.drone.Ixx = 7.5e-3;
param.drone.Iyy = 7.5e-3;
param.drone.Izz = 1.3e-2;
param.drone.I = diag([param.drone.Ixx param.drone.Iyy param.drone.Izz]);
param.drone.l = 0.23;
param.drone.m = 0.2; % ????????????????????? look up in doctoral thesis

param.drone.rotor.J = 6e-5;
param.drone.rotor.Kf = 3.13e-5;
param.drone.rotor.Km = 7.5e-7;

param.drone.Kt = diag([0.1 0.1 0.15]);

param.sp.psi0 = 0;

param.pid.Kd = 1;
param.pid.Kp = 1;

%% General parameters
param.env.g = 9.80665;
param.size_obs= 10;
param.amount_obs=30;
param.verti=12;

%% Initial conditions
param.q0 = [0 0 2]';
param.qdot0 = [0 0 0]';
