classdef Trimmer < handle
    % TRIMMER Class with functionality to trim a vehicle
    %
    
    properties
        x0, dx0, u0;
        ix, idx, iu;
        vehicle, gravity, environment, propulsion, aerodynamics, kinematics;
        Va;
        trim_state = [];
        trim_controls = [];
    end
    
    methods
        
        function obj = Trimmer(varargin)
            % TRIMMER Class constructor
            %
            % Syntax:  [obj] = Trimmer(sim_options)
            %          [obj] = Trimmer(sim_options, 10, 0.1, 50)
            %          [obj] = Trimmer(sim_options, 10, 0.1, 50, 'u0', [0.1, 0, 0.5, -0.2])
            %
            % Inputs:
            %    sim_options - Struct output from simulation_options, setting the trim options
            %    airspeed (optional) - Trim airspeed, in m/s
            %    gamma (optional) - Trim path angle, climb or descent, in rads
            %    r (optional) - Radius of trim turn. Positive for right turn. Inf for straight flight.
            %    u0 (optional) - Initialization input state for the search algorithm
            %
            % Outputs:
            %    obj - Class instance
            
            parser = inputParser();
            
            parser.addRequired('sim_options', @isstruct);
            parser.addOptional('airspeed', [], @isscalar);
            parser.addOptional('gamma', [], @isscalar);
            parser.addOptional('r', [], @isscalar);
            parser.addParameter('u0', [0; 0; 0.5; 0], @(x)size(x)==[4,1]);
            
            parser.parse(varargin{:});
            
            % Parse input arguments
            sim_options = parser.Results.sim_options;
            
            obj.Va = parser.Results.airspeed;
            if isempty(obj.Va)
                obj.Va = sim_options.controller.trim_airspeed;
            end
            if isempty(obj.Va)
                error('Undefined trim airspeed');
            end
            
            gamma = parser.Results.gamma;
            if isempty(gamma)
                gamma = sim_options.controller.trim_path_angle;
            end
            if isempty(gamma)
                error('Undefined trim path angle');
            end
            
            r = parser.Results.r;
            if isempty(r)
                r = sim_options.controller.trim_turn_radius;
            end
            if isempty(r)
                error('Undefined trim turn radius');
            end
            
            obj.u0 = parser.Results.u0;
            
            % Select model
            model_name = sim_options.vehicle;
            
            % Instantiate vehicle
            obj.vehicle = Vehicle(model_name);
            obj.vehicle.state.initialize(sim_options);
            
            % Generate the rest of the simulation components
            obj.gravity = Gravity(sim_options);
            obj.environment = Environment(sim_options);
            obj.propulsion = Propulsion(obj.vehicle);
            obj.aerodynamics = Aerodynamics(obj.vehicle);
            obj.kinematics = Kinematics(sim_options);
            
            % Setup optimization problem
            
            obj.x0 = [0; 0; 0; 0; gamma; 0; obj.Va; 0; 0; 0; 0; 0];
            obj.ix = [4 5 7 8 9 10 11 12]; % index of states of interest
            obj.u0 = obj.u0;
            obj.iu = [1 2 3 4]; % index of inputs of interest
            obj.dx0 = [0; 0; -obj.Va*sin(gamma); 0; 0; obj.Va/r; 0; 0; 0; 0; 0; 0];
            obj.idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12]; % index of derivatives of interest
            
        end
        
        function [solution, cost] = calc_trim(obj)
            % CALC_TRIM Solve the trimming problem
            % Uses the fminunc optimization routine to find a stable trim point, given the initialization arguments
            %
            % Syntax:  [obj] = calc_trim()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    solution - 12x1 vector containing the optimal optimization argument with contents:
            %    1: roll 2: pitch 3: u 4: v 5: w 6: p 7: q 8: r 9: aileron 10: elevator 11: throttle 12: rudder
            %    cost - Final optimization cost function value
            
            fprintf('Trimming aircraft...\n');
            
            % Setup initial optimization argument value
            argument_init = [obj.x0(obj.ix); obj.u0(obj.iu)]; % 8 states and 4 inputs
            options = optimoptions('fminunc');
            options.OptimalityTolerance = 1e-12; % From default 1e-6
            options.StepTolerance = 1e-12; % From default 1e-6
            [solution, cost, flag, output] = fminunc(@obj.cost_function, argument_init, options);
            
            % Store trimmed state
            obj.trim_state = VehicleState();
            obj.trim_state.set_vec_euler([solution(1); solution(2); 0]);
            obj.trim_state.set_vec_vel_linear_body([solution(3); solution(4); solution(5)]);
            obj.trim_state.set_vec_vel_angular_body([solution(6); solution(7); solution(8)]);
            % Store trimmed controls
            obj.trim_controls = [solution(9); solution(10); solution(11); solution(12)];
            
            fprintf('Trimming ended:\n');
            Va = norm(solution(3:5));
            fprintf('Achieved airspeed: %g\n', Va);
            
            fprintf('Trimmed control inputs: '); disp(solution(9:12)');
            
            results = cell(3,10);
            derivatives_actual = obj.kinematics.get_state_derivatives();
            x_dot_actual = [derivatives_actual.vec_pos_dot;...
                derivatives_actual.vec_euler_dot;...
                derivatives_actual.vec_vel_linear_body_dot;...
                derivatives_actual.vec_vel_angular_body_dot];
            results{1,1} = 'h_dot';     results{2,1} = obj.dx0(3);      results{3,1} = x_dot_actual(3);
            results{1,2} = 'phi_dot';   results{2,2} = obj.dx0(4);      results{3,2} = x_dot_actual(4);
            results{1,3} = 'theta_dot'; results{2,3} = obj.dx0(5);      results{3,3} = x_dot_actual(5);
            results{1,4} = 'psi_dot';   results{2,4} = obj.dx0(6);      results{3,4} = x_dot_actual(6);
            results{1,5} = 'u_dot';     results{2,5} = obj.dx0(7);      results{3,5} = x_dot_actual(7);
            results{1,6} = 'v_dot';     results{2,6} = obj.dx0(8);      results{3,6} = x_dot_actual(8);
            results{1,7} = 'w_dot';     results{2,7} = obj.dx0(9);      results{3,7} = x_dot_actual(9);
            results{1,8} = 'p_dot';     results{2,8} = obj.dx0(10);     results{3,8} = x_dot_actual(10);
            results{1,9} = 'q_dot';     results{2,9} = obj.dx0(11);     results{3,9} = x_dot_actual(11);
            results{1,10} = 'r_dot';    results{2,10} = obj.dx0(12);    results{3,10} = x_dot_actual(12);
            fprintf('Derivatives:\n');
            disp(results);
            fprintf('Final cost: %g\n', cost);
            fprintf('Optimization finished in %d iterations\n', output.iterations);
            temp_cell = cell2mat(results(2:3,:));
            disp(sum(temp_cell(1,:)-temp_cell(2,:),2)^2);
            
        end
        
        function state = get_trim_state(obj)
            % GET_TRIM_STATE Return the resulting trim state
            %
            % Syntax:  [state] = get_trim_state()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    state - A VehicleState instance, containing the resulting trimmed state
            
            if isempty(obj.trim_state)
                error('Call calc_trim() first');
            end
            
            state = obj.trim_state;
            
        end
        
        function output = get_trim_controls(obj)
            % GET_TRIM_CONTROLS Return the resulting trim controls
            %
            % Syntax:  [state] = get_trim_controls()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    state - A 4x1 array returning the static control vector, as initialized by the static_control input
            
            if isempty(obj.trim_controls)
                error('Call calc_trim() first');
            end
            
            output = obj.trim_controls;
            
        end
        
        
        function cost = cost_function(obj, argument)
            % COST_FUNCTION Cost function used in the fminunc optimization algorithm
            %
            % Syntax:  [cost] = calc_trim(argument)
            %
            % Inputs:
            %    argument - 12x1 vector containing the optimal optimization argument with contents:
            %    1: roll 2: pitch 3: u 4: v 5: w 6: p 7: q 8: r 9: aileron 10: elevator 11: throttle 12: rudder
            %
            % Outputs:
            %    cost - The corresponding error cost
            
            % vehicle state
            vec_pos = zeros(3,1); % position doesn't matter, isn't optimized
            obj.vehicle.state.set_vec_pos(vec_pos);
            vec_euler = [argument(1:2); 0]; % yaw doesn't matter, isn't optimized
            obj.vehicle.state.set_vec_euler(vec_euler);
            vec_vel_linear_body = argument(3:5); % all linear velocities matter and are optimized
            obj.vehicle.state.set_vec_vel_linear_body(vec_vel_linear_body);
            vec_vel_angular_body = argument(6:8); % all angular velocities matter and are optimized
            obj.vehicle.state.set_vec_vel_angular_body(vec_vel_angular_body);
            
            % vehicle input
            ctrl_input = argument(9:12); % All control inputs matter and are optimized
            
            % Calculate gravity
            obj.gravity.calc_gravity(obj.vehicle);
            vec_gravity_force_body = obj.gravity.get_force_body();
            
            % Calculate environment stuff
            obj.environment.calc_state(obj.vehicle);
            
            % Calculate propulsion
            obj.propulsion.calc_propulsion(obj.vehicle, obj.environment, ctrl_input);
            vec_propulsion_force_body = obj.propulsion.get_force_body();
            vec_propulsion_torque_body = obj.propulsion.get_torque_body();
            
            % Calculate aerodynamics
            obj.aerodynamics.calc_aerodynamics(obj.vehicle, obj.environment, ctrl_input);
            vec_aerodynamics_force_body = obj.aerodynamics.get_force_body();
            vec_aerodynamics_torque_body = obj.aerodynamics.get_torque_body();
            
            % Calculate derivatives
            vec_force_body = vec_gravity_force_body + vec_propulsion_force_body + vec_aerodynamics_force_body;
            vec_torque_body = vec_propulsion_torque_body + vec_aerodynamics_torque_body;
            obj.kinematics.set_wrench_body(vec_force_body,vec_torque_body);
            
            obj.kinematics.set_state(obj.vehicle.state);
            obj.kinematics.calc_state_derivatives(obj.vehicle);
            derivatives_actual = obj.kinematics.get_state_derivatives();
            
            x_dot_actual = [derivatives_actual.vec_pos_dot;...
                derivatives_actual.vec_euler_dot;...
                derivatives_actual.vec_vel_linear_body_dot;...
                derivatives_actual.vec_vel_angular_body_dot];
            
            % Caclulate cost
            derivatives_error = obj.dx0(obj.idx) - x_dot_actual(obj.idx);
            airspeed_error = obj.Va - norm(vec_vel_linear_body);
            derivatives_weight = 10^3*diag(length(derivatives_error));
            airspeed_weight = 1;
            cost = derivatives_error'*derivatives_weight*derivatives_error + airspeed_weight*airspeed_error^2;
            
        end
        
    end
    
end

