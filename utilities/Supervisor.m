classdef Supervisor < handle
% SUPERVISOR Simulation supervisor
% Contains, stores, handles and abstracts the simulation components. Mainly used to provide an API for the ODE
% calculation.
%
% Other m-files required: TODO
% MAT-files required: none
%
% See also: TODO

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
            
            obj.vehicle = Vehicle(sim_options.vehicle);
            obj.gravity = Gravity(sim_options);
            obj.environment = Environment(sim_options);
            obj.propulsion = Propulsion(obj.vehicle);
            obj.aerodynamics = Aerodynamics(obj.vehicle);
            obj.kinematics = Kinematics(sim_options);
            
            obj.temp_state = VehicleState();
            
        end
        
        function [] = initialize_sim_state(obj, sim_options)
            
            obj.vehicle.state.initialize(sim_options);
        end
        
        function [] = initialize_controller(obj, sim_options)
            
            obj.controller = Controller(sim_options);
        end
        
        function [] = sim_step(obj, t)
            
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
            
            % Calculate derivatives
            vec_force_body = vec_gravity_force_body + vec_propulsion_force_body + vec_aerodynamics_force_body;
            vec_torque_body = vec_propulsion_torque_body + vec_aerodynamics_torque_body;
            obj.kinematics.set_wrench_body(vec_force_body,vec_torque_body);
            
            obj.kinematics.set_state(obj.vehicle.state);
            obj.kinematics.calc_state_derivatives(obj.vehicle);
            
        end
        
        function [] = update_vehicle_state_vector(obj, state_vector)
            
            obj.temp_state.set_state_vector(state_vector);
            obj.vehicle.set_state(obj.temp_state);
            
        end
        
        function [] = integrate_fe(obj)
            obj.kinematics.integrate_fe();
            obj.kinematics.write_state(obj.temp_state);
            obj.vehicle.set_state(obj.temp_state);
        end
        
        function [state_derivatives] = ode_eval(obj, t, state_vector)
            
            obj.update_vehicle_state_vector(state_vector);
            state_derivatives = obj.kinematics.get_state_derivatives_serial();
            
        end
        
    end
    
end

