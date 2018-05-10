


load_path;

%% Initialize the simulation

% set simulation options
sim_options = sim_options();

% select model
model = skywalker();

% instantiate it
vehicle = Vehicle(model);
vehicle.state.initialize(sim_options);

% generate the rest of the simulation components
gravity = Gravity(sim_options);
environment = Environment(sim_options);
propulsion = Propulsion(vehicle);
aerodynamics = Aerodynamics(vehicle);
kinematics = Kinematics();

% select visualization options

%% Begin simulation
% for each loop:

ctrl_input = zeros(4,1);

% Calculate gravity
vec_gravity_force_body = gravity.get_gravity_force(vehicle);

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

% Integrate kinematics and update state
kinematics.set_wrench_body(vec_force_body,vec_torque_body);
state_derivatives = kinematics.get_state_derivatives;
vehicle_state_new = kinematics.integrate(state_derivatives);
vehicle.set_state(vehicle_state_new);

% Update visual output