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
        array_inputs; % The whole input history
        indx_array_inputs=1;
        save_inputs;
        array_states; % The whole state history
        indx_array_states=1;
        save_states;
        array_time; % The whole time array
        
        solver_type;
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
            
            obj.solver_type = sim_options.solver.solver_type;
            % Initialize states and input arrays
            if sim_options.solver.solver_type == 0
            % Time vector is known
                
                % Retrieve the time vector parameters
                t_0 = sim_options.solver.t_0;
                t_f = sim_options.solver.t_f;
                dt = sim_options.solver.dt;
                
                % Initialize saved signals
                num_frames = (t_f-t_0)/dt;
                if sim_options.record_states
                    obj.save_states = true;
                    temp_state = obj.vehicle.state.serialize();
                    obj.array_states = zeros(size(temp_state,1),num_frames);
                end
                if sim_options.record_inputs
                    obj.save_inputs = true;
                    obj.array_inputs = zeros(4,num_frames);
                end
                
            elseif ismember(sim_options.solver.solver_type, [1 2])
            % Time vector size is unknown, initialize empty
            
                if sim_options.record_states
                    obj.save_states = true;
                end
                if sim_options.record_inputs
                    obj.save_inputs = true;
                end
                obj.array_states = [];
                obj.array_inputs = [];
                obj.array_time = [];
                
            else
                error('Unuspported solver type %d specified', sim_options.solver.solver_type);
            end
            
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
            
            % Record resulting inputs
            if obj.save_inputs
                if obj.solver_type == 0
                % Time vector is known
                    obj.array_inputs(:,obj.indx_array_inputs) = obj.ctrl_input;
                    obj.indx_array_inputs = obj.indx_array_inputs + 1;
                elseif ismember(obj.solver_type, [1 2])
                    obj.store_time(t);
                    obj.store_inputs(obj.ctrl_input);
                else
                    error('Unuspported solver type %d specified', sim_options.solver.solver_type);
                end
            end
            
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
            
            % Record resulting states
            if obj.save_states
                if obj.solver_type == 0
                % Time vector is known
                    obj.array_states(:,obj.indx_array_states) = obj.vehicle.state.serialize();
                    obj.indx_array_states = obj.indx_array_states + 1;
                elseif ismember(obj.solver_type, [1 2])
                % State recording is delegated to the ode solver itself
                else
                    error('Unuspported solver type %d specified', sim_options.solver.solver_type);
                end
            end
            
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
            obj.sim_step(t);
            state_derivatives = obj.kinematics.get_state_derivatives_serial();
            
        end
        
        function [status] = ode_outputFcn(obj, t, y, flag)
            % ODE_OUTPUTFCN Callback from each valid ode evaluation
            % Used to orchestrate multiple functionalities.
            %
            % Syntax:  [] = ode_outputFcn(t, y, flag) (Not to be called manually)
            %
            % Inputs:
            %    t - Simulation time, in seconds
            %    y - The ode state, typically a 12x1 vector with contents: 3x1 position, 3x1 Euler, 3x1 linear velocity,
            %    3x1 angular velocity
            %    flag - The solver state, 'init', [] or 'done'
            %
            % Outputs:
            %    state_derivatives - a 12x1 vector with contents: 3x1 position derivative, 3x1 Euler angle 
            %    derivatives, 3x1 linear velocity derivatives, 3x1 angular velocity derivatives
            
            
            if isempty(flag)
                % Normal ode step
                
                status = 0; % Continue solving
                
            elseif flag=='init'
                % ode initializatin, nothing to do here
                
            elseif flag == 'done'
                % ode ended
                
                % Nothing to do here
                
            end
                
        end
        
        function [] = store_time(obj, t)
            % STORE_TIME Append a time to the time storage array
            %
            % Syntax:  [] = store_time(t)
            %
            % Inputs:
            %    t - A time instance
            %
            % Outputs:
            %    (none)
            
            obj.array_time(end+1) = t;
            
        end
        
        function [] = store_inputs(obj, ctrl_input)
            % STORE_INPUTS Append a control vector to the storage array.
            %
            % Syntax:  [] = store_inputs(ctrl_input)
            %
            % Inputs:
            %    ctrl_input - A 4x1 vector containing commands for Aileron [-1, 1], elevator [-1, 1], throttle [0, 1], rudder [-1, 1]
            %
            % Outputs:
            %    (none)
            
            obj.array_inputs(:,end+1) = ctrl_input;
            
        end
        
    end
    
end

