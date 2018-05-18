function [] = plot_trajectory(sim_output, sim_options, varargin)
% PLOT_TRAJECTORY Batch-plot the simulated trajectory output
% Generate a 3D plot of the position trajectory of the vehicle. Can be used after the end of the simulation.
%
% Syntax:  [] = plot_trajectory(sim_output, sim_options)
%          [] = plot_trajectory(sim_output, sim_options, 'plot_vehicle', true)
%          [] = plot_trajectory(sim_output, sim_options, 'plot_vehicle', true, 'num_graphics', 20)
%
% Inputs:
%    sim_output - Struct output from the simulation script
%    sim_options - Struct output from the simulation_options function
%    varargin - other optional name-value pairs:
%       plot_vehicle - request the vehicle graphic to be plotted on the trajectory (default false)
%       num_graphics - if plot_vehicle is set to true, it sets the number of equidistant graphics to plot (default 10)
%
% Outputs:
%    (none)
%
% Other m-files required: simulation, simulation_options, draw_aircraft_body, Vehicle, VehicleState
% Subfunctions: none
% MAT-files required: none
%
% See also: simulation, simulation_options

% Created at 2018/02/18 by George Zogopoulos-Papaliakos

% Parse plot options
parser = inputParser;
parser.addParameter('plot_vehicle',false,@islogical); % Draw the 3D model of the vehicle
parser.addParameter('num_graphics',10,@isPositiveIntegerValuedNumeric); % How many 3D models of the vehicle will be plotted
parser.parse(varargin{:});

% process inputs to function
states = sim_output.array_states;
pn = states(1,:); % inertial North position
pe = states(2,:); % inertial East position
pd = states(3,:); % Down coordinate
pz = -pd;

% Generate figure
figure_handle = figure();
% axes_handle=gca;

% Plot the trajectory
line_axis = plot3(pe,pn,pz);
hold on;
grid on;

xlabel('East (m)')
ylabel('North (m)')
zlabel('-Down (m)');

% Draw the vehicle
if parser.Results.plot_vehicle
    % Find the plotting frames
    num_frames = size(pn,2);
    frames_to_plot = floor(linspace(1, num_frames, parser.Results.num_graphics));
    
    % Select model
    model_name = sim_options.vehicle;
    eval(sprintf("model = %s();", model_name));
    % Set graphic
    graphic_name = model.graphic;
    eval(sprintf("graphic = %s();", graphic_name));
    
    % Instantiate a vehicle
    vehicle = Vehicle(model);
    
    for index=frames_to_plot
        % Place the vehicle in the correct position and orientation
        vehicle.state.set_vec_pos(states(1:3, index));
        vehicle.state.set_vec_euler(states(4:6, index));
        
        % Draw the vehicle
        draw_aircraft_body(vehicle, graphic, []);
    end
end

axis equal

end