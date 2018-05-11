function [ sim_options ] = simulation_options( )
%SIMULATION_OPTIONS Summary of this function goes here
%   Detailed explanation goes here

sim_options.dt = 0.001;

%% Initialization options

sim_options.init.vec_pos = [0;0;0];
sim_options.init.vec_euler = [0;0;0];
sim_options.init.vec_vel_linear_body = [10;0;0];
sim_options.init.vec_vel_angular_body = [0;0;0];

%% Visualization options

sim_options.visualization.draw_graphics = true;
sim_options.visualization.draw_forces = true;
sim_options.visualization.draw_states = true;

%% Gravity options

sim_options.gravity.g_0 = 9.81;

%% Environment options

sim_options.environment.model_type = 1;

sim_options.environment.rho_0 = 1.225;

%% Wind options

sim_options.environment.wind = [0;0;0]; % North, East, Down, in m/s

end

