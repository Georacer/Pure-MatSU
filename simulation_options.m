function [ sim_options ] = simulation_options( )
%SIMULATION_OPTIONS User-selectable simulation options (non vehicle-specific)
%Change any line in this file to contorl the simulation initialization and behaviour
%
% Syntax:  [sim_options] = simulation_options()
%
% Inputs:
%    (none)
%
% Outputs:
%    sim_options - struct variable containing simulation options fields
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: skywalker_2013

% Created at 2018/02/15 by George Zogopoulos-Papaliakos

%% General simulation options

% Time step (s)
sim_options.dt = 0.001;
% Start time (s)
sim_options.t_0 = 0;
% End time (s)
sim_options.t_f = 10;

% Simulated vehicle
sim_options.vehicle = 'skywalker_2013';

% Save vehicle states
sim_options.record_states = true;
% Save control inputs
sim_options.record_inputs = true;

%% Controller options

% Choose controller. Available options:
% 0: Constant zero output
sim_options.controller.type = 0;

%% Initialization options

% Vehicle starting position (north, east, down, in m)
sim_options.init.vec_pos = [0;0;0];
% Vehicle starting Euler angles (in rad)
sim_options.init.vec_euler = [0;0;0];
% Vehicle starting velocity (body frame, in m/s)
sim_options.init.vec_vel_linear_body = [10;0;0];
% Vehicle starting angular velocity (body frame, in rad/s)
sim_options.init.vec_vel_angular_body = [0;0;0];

%% Visualization options
% CAUTION: Significantly slow the simulation down

% Draw a live 3D graphic of the vehicle (about 60x slower)
sim_options.visualization.draw_graphics = false;
% Live plot of the forces exerted on the vehicle (about 1000x slower)
sim_options.visualization.draw_forces = false;
% Live plot of the vehicle states (about 10x slower)
sim_options.visualization.draw_states = false;

%% Gravity options

% Nominal gravity value (in m/s^2)
sim_options.gravity.g_0 = 9.81;

%% Environment options

% Select the environment model (currently only 1 available)
sim_options.environment.model_type = 1;
% Nominal air density (in kg/m^3)
sim_options.environment.rho_0 = 1.225;

%% Wind options

% Static wind vector (north, east, down, in m/s)
sim_options.environment.wind = [0;0;0];

end

