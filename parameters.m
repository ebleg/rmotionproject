%% PARAMETERS
% Define parameters for simulation and control
param = struct();

%% Environmental properties
param.env.rho = 1.225; % Air density
param.env.g = 9.80665; % Gravitational acceleration

%% Drone physical properties
param.drone.m = 2;  % Drone mass
param.drone.I = diag([0.5829 0.7169 1.].*1e-4);  % Drone moment of inertia
param.drone.r = [0.1 0.1 0.1 0.1]; % Distance between c.m. of the drone and rotor centres

%% Rotor properties
param.drone.rotor.kT = 1;   % Rotor thrust coefficient
param.drone.rotor.kQ = 1;   % Rotor torque coefficient
param.drone.rotor.D = 0.07;  % Rotor diameter
