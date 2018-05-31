classdef Supervisor < handle
% SUPERVISOR Simulation supervisor
% Contains, stores, handles and abstracts the simulation components. Mainly used to provide an API for the ODE
% calculation.
%
% Other m-files required: Vehicle, Gravity, Environment, Propulsion, Aerodynamics, Kinematics, Controller, VehicleState
% MAT-files required: none
%
% See also: Vehicle, Gravity, Environment, Propulsion, Aerodynamics, Kinematics, Controller, simulation

% Created at 2018/05/31 by George Zogopoulos-Papaliakos
    
    properties
        vehicle; % Vehicle object
        gravity; % Gravity object
        environment; % Environment object
        propulsion; % Propulsion object
        aerodynamics; % Aerodynamics object
        kinematics; % Kinematics object
        controller; % Controller object
        
        temp_state; % VehicleState object
        ctrl_input; % The control input vector
    end
    
    methods
        
        function obj = Supervisor(sim_options)
            % SUPERVISOR Class constructor
            % ATTENTION: The object returned is not ready for use yet. The simulation script must decide on the initial
            % state and trim the control inputs if needed.
            %
            % Syntax:  [obj] = Supervisor(sim_options)
            %
            % Inputs:
            %    sim_options - Struct output from simulation_options, setting the trim options
            %
            % Outputs:
            %    obj - Class instance
            
            % Instantiate simulation components
            obj.vehicle = Vehicle(sim_options.vehicle);
            obj.gravity = Gravity(sim_options);
            obj.environment = Environment(sim_options);
            obj.propulsion = Propulsion(obj.vehicle);
            obj.aerodynamics = Aerodynamics(obj.vehicle);
            obj.kinematics = Kinematics(sim_options);
            
            obj.temp_state = VehicleState();
            
        end
        
        function [] = initialize_sim_state(obj, sim_options)
            % INITIALIZE_SIM_STATE Set the initial simulation state
            %
            % Syntax:  [] = initialize_sim_state(sim_options)
            %
            % Inputs:
            %    sim_options - Struct output from simulation_options, setting the trim options. Uses the
            %    sim_options.init structure.
            %
            % Outputs:
            %    (none)
            
            obj.vehicle.state.initialize(sim_options);
            
        end
        
        function [] = initialize_controller(obj, sim_options)
            % INITIALIZE_CONTROLLER Instantiate the vehicle controller
            %
            % Syntax:  [] = initialize_controller(sim_options)
            %
            % Inputs:
            %    sim_options - Struct output from simulation_options, setting the trim options. Uses the
            %    sim_options.controller structure.
            %
            % Outputs:
            %    (none)
            
            obj.controller = Controller(sim_options);
            
        end
        
        function [] = sim_step(obj, t)
            % SIM_STEP Evaluate the state derivatives
            % Orchestrates the simulation components to calculate the vehicle state derivatives. Uses the stored
            % vehicle.state to perform the derivatives calculation. Saves the derivatives in the kinematics object.
            %
            % Syntax:  [] = sim_step(t)
            %
            % Inputs:
            %    t - Simulation time, for passing onto the controller
            %
            % Outputs:
            %    (none)
            
            % Generate controller output, based on previous state
            obj.ctrl_input = obj.controller.calc_output(obj.vehicle.state, t);
            
            % Calculate gravity
            obj.gravity.calc_gravity(obj.vehicle);
            vec_gravity_force_body = obj.gravity.get_force_body();
            
            % Calculate environment stuff
            obj.environment.calc_state(obj.vehicle);
            
            % Calculate propulsion
            obj.propulsion.calc_propulsion(obj.vehicle, obj.environment, obj.ctrl_input);
            vec_propulsion_force_body = obj.propulsion.get_force_body();
            vec_propulsion_torque_body = obj.propulsion.get_torque_body();
            
            % Calculate aerodynamics
            obj.aerodynamics.calc_aerodynamics(obj.vehicle, obj.environment, obj.ctrl_input);
            vec_aerodynamics_force_body = obj.aerodynamics.get_force_body();
            vec_aerodynamics_torque_body = obj.aerodynamics.get_torque_body();
            
            % Calculate the overall applied wrench
            vec_force_body = vec_gravity_force_body + vec_propulsion_force_body + vec_aerodynamics_force_body;
            vec_torque_body = vec_propulsion_torque_body + vec_aerodynamics_torque_body;
            obj.kinematics.set_wrench_body(vec_force_body,vec_torque_body);
            
            % Set the state which the kinematics will use for the derivative calculation
            obj.kinematics.set_state(obj.vehicle.state);
            % Perform the kinematics derivative calculation
            obj.kinematics.calc_state_derivatives(obj.vehicle);
            
        end
        
        function [] = update_vehicle_state_vector(obj, state_vector)
            % UPDATE_VEHICLE_STATE_VECTOR Setter of the vehicle state, given a state in vector form
            %
            % Syntax:  [] = update_vehicle_state_vector(state_vector)
            %
            % Inputs:
            %    state_vector - a 12x1 vector with contents: 3x1 position, 3x1 Euler, 3x1 linear velocity,
            %    3x1 angular velocity
            %
            % Outputs:
            %    (none)
            
            % Copy the state_vector onto the local VehicleState object
            obj.temp_state.set_state_vector(state_vector);
            % Set the vehicle state to the temp_state
            obj.vehicle.set_state(obj.temp_state);
            
        end
        
        function [] = integrate_fe(obj)
            % INTEGRATE_FE Perform a Forward-Euler integration step of the simulation
            %
            % Syntax:  [] = integrate_fe()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    (none)
            
            % Use the kinematics member to integrate. Uses the internally saved vehicle state
            obj.kinematics.integrate_fe();
            % Write the new state to a temporary VehicleState object
            obj.kinematics.write_state(obj.temp_state);
            % Pass the new state onto the vehicle member.
            obj.vehicle.set_state(obj.temp_state);
        end
        
        function [state_derivatives] = ode_eval(obj, t, state_vector)
            % ODE_EVAL Method to provide the 'odefun' functionality, for use with an ode solver (e.g. ode45)
            %
            % Syntax:  [state_derivatives] = ode_eval(t, state_vector)
            %
            % Inputs:
            %    t - Simulation time, in seconds
            %    state_vector - a 12x1 vector with contents: 3x1 position, 3x1 Euler, 3x1 linear velocity,
            %    3x1 angular velocity
            %
            % Outputs:
            %    state_derivatives - a 12x1 vector with contents: 3x1 position derivative, 3x1 Euler angle 
            %    derivatives, 3x1 linear velocity derivatives, 3x1 angular velocity derivatives
            
            obj.update_vehicle_state_vector(state_vector);
            state_derivatives = obj.kinematics.get_state_derivatives_serial();
            
        end
        
    end
    
end

