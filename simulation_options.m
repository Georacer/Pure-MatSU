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

% Simulated vehicle
sim_options.vehicle = 'skywalker_2013';

% Save vehicle states
sim_options.record_states = true;
% Save control inputs
sim_options.record_inputs = true;

%% ODE solver options

% Time step (s)
sim_options.solver.dt = 0.001; % (Not used if ode_solver!=0)
% Start time (s)
sim_options.solver.t_0 = 0;
% End time (s)
sim_options.solver.t_f = 30;
% Time accuracy
sim_options.solver.t_eps = 1e-6; % in seconds

% Select the differential equations solver method
% Available options:
% 0 - Forward Euler, allows very fine time resolution
% 1 - Matlab's ode45, a good balane between resolution and speed
% 2 - Matlab's ode15s, fast but maybe with crude resolution
sim_options.solver.solver_type = 1;

%% Controller options

% Choose controller. Available options:

% % 0: Constant output
% sim_options.controller.type = 0;
% sim_options.controller.static_output = [0; 0; 0; 0]; % Aileron [-1, 1], elevator [-1, 1], throttle [0, 1], rudder [-1, 1]

% % 1: Constant, trimmed output.
sim_options.controller.type = 1;
sim_options.controller.trim_airspeed = 10; % Trim airspeed (in m/s)
sim_options.controller.trim_path_angle = 0.1; % Trim flight path angle (in radians)
sim_options.controller.trim_turn_radius = 50; % Trim turn radius. Positive for right turn (in m). inf for straight line.

%% Initialization options

% Vehicle starting position (north, east, down, in m)
sim_options.init.vec_pos = [0 ; 0; 0];
% Vehicle starting Euler angles (in rad). (Not effective with controller type = 1)
sim_options.init.vec_euler = [0;0;0];
% Vehicle starting velocity (body frame, in m/s). (Not effective with controller type = 1)
sim_options.init.vec_vel_linear_body = [10;0;0];
% Vehicle starting angular velocity (body frame, in rad/s). (Not effective with controller type = 1)
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

