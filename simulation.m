
close all;
clear;
clc;

% load_path;

%% Initialize the simulation

% set simulation options
sim_options = simulation_options();

% select model
model = skywalker_2013();
graphic = airplane_1();

% instantiate it
vehicle = Vehicle(model);
vehicle.state.initialize(sim_options);

% generate the rest of the simulation components
gravity = Gravity(sim_options);
environment = Environment(sim_options);
propulsion = Propulsion(vehicle);
aerodynamics = Aerodynamics(vehicle);
kinematics = Kinematics(sim_options);
vehicle_state_new = VehicleState(); % Use a swap variable to update vehicle state

% initialize visualization
if sim_options.visualization.draw_graphics
    draw_aircraft(vehicle, graphic, true);    
end
if sim_options.visualization.draw_forces
    plot_forces(gravity, propulsion, aerodynamics, 0, true);    
end
if sim_options.visualization.draw_states
    plot_states(vehicle, 0, true);    
end

%% Begin simulation

t_0 = 0;
t_f = 10;
dt = sim_options.dt;
t = t_0;

% for each loop:
while (t<t_f)
    
    ctrl_input = zeros(4,1);
    
    % Calculate gravity
    gravity.calc_gravity(vehicle);
    vec_gravity_force_body = gravity.get_force_body();
    
    % Calculate environment stuff
    environment.calc_state(vehicle);
    
    % Calculate propulsion
    propulsion.calc_propulsion(vehicle, environment, ctrl_input);
    vec_propulsion_force_body = propulsion.get_force_body();
    vec_propulsion_torque_body = propulsion.get_torque_body();
    
    % Calculate aerodynamics
    aerodynamics.calc_aerodynamics(vehicle, environment, ctrl_input);
    vec_aerodynamics_force_body = aerodynamics.get_force_body();
    vec_aerodynamics_torque_body = aerodynamics.get_torque_body();
    
    % Calculate derivatives
    vec_force_body = vec_gravity_force_body + vec_propulsion_force_body + vec_aerodynamics_force_body;
    vec_torque_body = vec_propulsion_torque_body + vec_aerodynamics_torque_body;    
    kinematics.set_wrench_body(vec_force_body,vec_torque_body);
    
    kinematics.set_state(vehicle.state);
    kinematics.calc_state_derivatives(vehicle);
    
%     % Debug output
%     vehicle_state = kinematics.get_state();
%     state_derivatives = kinematics.get_state_derivatives();
%     fprintf('*** DEBUG OUTPUT @ t=%f***\n',t);
%     disp('Position')
%     disp(vehicle.state.get_vec_pos()');
%     disp('Position derivative')
%     disp(state_derivatives.vec_pos_dot');
%     disp('Orientation')
%     disp(vehicle.state.get_vec_euler()');
%     disp('Orientation derivative')
%     disp(state_derivatives.vec_euler_dot');
%     disp('Linear velocity')
%     disp(vehicle.state.get_vec_vel_linear_body()');
%     disp('Linear velocity derivative')
%     disp(state_derivatives.vec_vel_linear_body_dot');
%     disp('Angular velocity')
%     disp(vehicle.state.get_vec_vel_angular_body()');
%     disp('Angular velocity derivative')
%     disp(state_derivatives.vec_vel_angular_body_dot');
%     disp('Airdata')
%     disp(vehicle.get_airdata(environment)');
%     disp('');
    
    % Integrate kinematics
    kinematics.integrate();
    
    % Update vehicle state
    kinematics.write_state(vehicle_state_new);
    vehicle.set_state(vehicle_state_new);
    
    % Update visual output
    if sim_options.visualization.draw_graphics
        draw_aircraft(vehicle, graphic, false);
    end
    if sim_options.visualization.draw_forces
        plot_forces(gravity, propulsion, aerodynamics, t, false);
    end
    if sim_options.visualization.draw_states
        plot_states(vehicle, t, false);
    end
    
    t = t + dt;
    
end
