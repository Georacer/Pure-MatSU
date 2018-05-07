function [ sim_options ] = simulation_options( )
%SIMULATION_OPTIONS Summary of this function goes here
%   Detailed explanation goes here

sim_options.dt = 0.001;

%% Gravity options


%% Environment options

sim_options.environment.model_type = 1;

sim_options.environment.rho_0 = 1.225;

%% Wind options


end

