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
        
        t; % Simulation time
        t_f; % Final time
        
        temp_state; % VehicleState object
        ctrl_input; % The control input vector
        array_inputs; % The whole input history
        indx_array_inputs=1;
        save_inputs;
        array_states; % The whole state history
        indx_array_states=1;
        save_states;
        array_time_states; % Time array, referring to state evaluations
        array_time_inputs; % Time array, referring to control input evaluations
        
        solver_type;
        
        draw_graphics; % Live draw graphics flag
        draw_forces; % Live draw forces flag
        draw_states; % Live draw states flag
        
        waitbar_handle;
        frame_skip=100; % Every how many frames the waitbar will update;
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
                    obj.array_time_states = zeros(1, num_frames);
                end
                if sim_options.record_inputs
                    obj.save_inputs = true;
                    obj.array_inputs = zeros(4, num_frames);
                    obj.array_time_inputs = zeros(1, num_frames);
                end
                
            elseif ismember(sim_options.solver.solver_type, [1 2])
            % Time vector size is unknown, initialize empty
            
                if sim_options.record_states
                    obj.save_states = true;
                    obj.array_states = [];
                    obj.array_time_states = [];
                end
                if sim_options.record_inputs
                    obj.save_inputs = true;
                    obj.array_inputs = [];
                    obj.array_time_inputs = [];
                end
                
            else
                error('Unuspported solver type %d specified', sim_options.solver.solver_type);
            end
            
            % Initialize waitbar
            obj.waitbar_handle = waitbar(0, 'Simulation running...');
            obj.t_f = sim_options.solver.t_f;
            
            % Decide if live graphics and plots are to be drawn
            obj.draw_graphics = sim_options.visualization.draw_graphics;
            obj.draw_forces = sim_options.visualization.draw_forces;
            obj.draw_states = sim_options.visualization.draw_states;
            
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
            
            % Initialize visualization if needed
            if obj.draw_graphics
                draw_aircraft(obj.vehicle, true);
            end
            % Initialize forces plots if needed
            if obj.draw_forces
                plot_forces(obj.gravity, obj.propulsion, obj.aerodynamics, 0, true);
            end
            % Initialize states plots if needed
            if obj.draw_states
                plot_states(obj.vehicle, 0, true);
            end
            
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
            
            % Store simulation time
            obj.t = t;
            
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
            
            % Update waitbar
            if obj.solver_type == 0
                if mod(obj.indx_array_states, obj.frame_skip)==0
                    waitbar(t/obj.t_f, obj.waitbar_handle);
                end
            elseif ismember(obj.solver_type, [1 2])
                waitbar(t/obj.t_f, obj.waitbar_handle);
            end
            
            % Record resulting inputs
            if obj.save_inputs
                if obj.solver_type == 0
                	obj.store_time_inputs(t, obj.indx_array_inputs); % Store input calculation time
                    obj.store_inputs(obj.ctrl_input, obj.indx_array_inputs); % Store control input
                    obj.indx_array_inputs = obj.indx_array_inputs + 1;
                elseif ismember(obj.solver_type, [1 2])
                    % Do nothing, because ode solvers may do dummy calls of the system function. Use outputFcn instaed.
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
                    obj.store_time_states(obj.t, obj.indx_array_states); % Store time instance
                    obj.store_states(obj.vehicle.state.serialize(), obj.indx_array_states); % Store state vector
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
                
                % Save control inputs
                obj.store_time_inputs(obj.t); % Save time instance. Cannot pass t argument, because it may be vector
                obj.store_inputs(obj.ctrl_input); % Save contorl vector
                
                % Draw live output
                if obj.draw_graphics
                    draw_aircraft(obj.vehicle, false);
                    pause(0.001); % Allow some time for patches to render
                end
                
                % Draw live forces plots
                if obj.draw_forces
                    plot_forces(obj.gravity, obj.propulsion, obj.aerodynamics, t, false);
                end
                
                % Draw live states plots
                if obj.draw_states
                    plot_states(obj.vehicle, t, false);
                end
                
                status = 0; % Continue solving
                
            elseif flag=='init'
                % ode initializatin, nothing to do here
                
            elseif flag == 'done'
                % ode ended
                
                % Close the waitbar
                obj.close_waitbar();
                
            end
                
        end
        
        function [] = store_time_states(obj, t, index)
            % store_time_states Append a time to the time storage array, referring to state evaluations
            % If called with only the first argument, will append the provided time to the end of the array. If the size
            % of array_time_states is known and the array is preallocated, then use the index argument to select the
            % storage location.
            %
            % Syntax:  [] = store_time_states(t)
            %          [] = store_time_states(t, index)
            %
            % Inputs:
            %    t - A time instance
            %    index - The array index at which t is to be stored.
            %
            % Outputs:
            %    (none)
            
            if nargin<3            
                obj.array_time_states(end+1) = t;
            else
                obj.array_time_states(index) = t;
            end
            
        end
        
        function [] = store_states(obj, state_vector, index)
            % STORE_states Append a state vector to the storage array.
            % If called with only the first argument, will append the provided state to the end of the array. If the size
            % of array_states is known and the array is preallocated, then use the index argument to select the
            % storage location.
            %
            % Syntax:  [] = store_states(state_vector)
            %          [] = store_states(state_vector, index)
            %
            % Inputs:
            %    state_vector - a 12x1 vector with contents: 3x1 position, 3x1 Euler, 3x1 linear velocity,
            %    3x1 angular velocity
            %    index - The array index at which t is to be stored.
            %
            % Outputs:
            %    (none)
            
            if nargin<3            
                obj.array_states(end+1) = state_vector;
            else
                obj.array_states(:, index) = state_vector;
            end
            
        end
            
        function [] = store_time_inputs(obj, t, index)
            % store_time_inputs Append a time to the time storage array, referring to control input evaluations
            % If called with only the first argument, will append the provided time to the end of the array. If the size
            % of array_time_inputs is known and the array is preallocated, then use the index argument to select the
            % storage location.
            %
            % Syntax:  [] = store_time_inputs(t)
            %          [] = store_time_inputs(t, index)
            %
            % Inputs:
            %    t - A time instance
            %    index - The array index at which t is to be stored.
            %
            % Outputs:
            %    (none)
            
            if nargin<3
                obj.array_time_inputs = [obj.array_time_inputs t];
            else
                if ~isscalar(t)
                    error('Cannot handle non-scalar time value');
                end
                obj.array_time_inputs(index) = t;
            end
            
        end
        
        function [] = store_inputs(obj, ctrl_input, index)
            % STORE_INPUTS Append a control vector to the storage array.
            % If called with only the first argument, will append the provided input to the end of the array. If the size
            % of array_inputs is known and the array is preallocated, then use the index argument to select the
            % storage location.
            %
            % Syntax:  [] = store_inputs(ctrl_input)
            %          [] = store_inputs(ctrl_input, index)
            %
            % Inputs:
            %    ctrl_input - A 4x1 vector containing commands for Aileron [-1, 1], elevator [-1, 1], throttle [0, 1], rudder [-1, 1]
            %    index - The array index at which t is to be stored.
            %
            % Outputs:
            %    (none)
            
            if nargin<3            
                obj.array_inputs = [obj.array_inputs ctrl_input];
            else
                obj.array_inputs(:, index) = ctrl_input;
            end
            
        end
        
        function [] = close_waitbar(obj)
            % CLOSE_WAITBAR Close the waitbar object
            %
            % Syntax:  [] = close_waitbar()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    (none)
            
            close(obj.waitbar_handle);
            
        end
        
    end
    
end

