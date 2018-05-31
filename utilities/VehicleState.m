classdef VehicleState < handle
% VEHICLESTATE Container class for the general vehicle state
% Provides unified getter-setter API for all vehicle types
%
% Other m-files required: simulation_options
% MAT-files required: none
%
% See also: Vehicle, simulation_options

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        
        vec_pos;
        vec_euler;
        vec_vel_linear_body;
        vec_vel_angular_body;
        
        num_states = 12;
        
    end
    
    methods
        
        function obj = VehicleState()
            % VEHICLESTATE Class constructor
            %
            % Syntax:  [obj] = VehicleState()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    obj - Class instance
           
            obj.vec_pos = zeros(3,1);
            obj.vec_euler = zeros(3,1);
            obj.vec_vel_linear_body = zeros(3,1);
            obj.vec_vel_angular_body = zeros(3,1);
            
        end
        
        function initialize(obj, sim_options)
            % INITIALIZE Initialize the vehicle state
            % Initializes a vehicle state based on the provided simulation options struct
            %
            % Syntax:  [obj] = initialize(sim_options)
            %
            % Inputs:
            %    sim_options - struct output from simulation_options function
            %
            % Outputs:
            %    (none)
            
            obj.vec_pos = sim_options.init.vec_pos;
            obj.vec_euler = sim_options.init.vec_euler;
            obj.vec_vel_linear_body = sim_options.init.vec_vel_linear_body;
            obj.vec_vel_angular_body = sim_options.init.vec_vel_angular_body;
            
        end
        
        function set_state(obj, external_state)
            % SET_STATE Vehicle state setter
            % Uses the a passed VehicleState object, by copying its contents
            %
            % Syntax:  [] = set_state(model)
            %
            % Inputs:
            %    external_state - A VehicleState instance
            %
            % Outputs:
            %    (none)
            
            obj.set_vec_pos(external_state.get_vec_pos());
            obj.set_vec_euler(external_state.get_vec_euler());
            obj.set_vec_vel_linear_body(external_state.get_vec_vel_linear_body());
            obj.set_vec_vel_angular_body(external_state.get_vec_vel_angular_body());
            
        end
        
        function set_state_vector(obj, external_state_vector)
            % SET_STATE_VETOR Vehicle state setter
            % Uses the a passed vector containing the vehicle state
            %
            % Syntax:  [] = set_state(model)
            %
            % Inputs:
            %    external_state_vector - a 12x1 vector with contents: 3x1 position derivative, 3x1 Euler angle 
            %    derivatives, 3x1 linear velocity derivatives, 3x1 angular velocity derivatives
            %
            % Outputs:
            %    (none)
            
            obj.set_vec_pos(external_state_vector(1:3));
            obj.set_vec_euler(external_state_vector(4:6));
            obj.set_vec_vel_linear_body(external_state_vector(7:9));
            obj.set_vec_vel_angular_body(external_state_vector(10:12));
            
        end
        
        function vec_pos = get_vec_pos(obj)
            % GET_VEC_POS - Returns the vehicle position vector
            %
            % Syntax:  [vec_pos] = get_vec_pos()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_pos - a 3x1 array containing the North, East and Down vehicle position (in SI units)
            
            vec_pos = obj.vec_pos;
            
        end
        
        function set_vec_pos(obj, vec_pos)
            % SET_VEC_POS - Sets the vehicle position
            %
            % Syntax:  [] = set_vec_pos(vec_pos)
            %
            % Inputs:
            %    vec_pos - a 3x1 array containing the North, East and Down vehicle position (in SI units)
            %
            % Outputs:
            %    (none)
            
            obj.vec_pos = vec_pos;
            
        end
        
        function vec_euler = get_vec_euler(obj)
            % GET_EULER_POS - Returns the vehicle orientation
            %
            % Syntax:  [vec_euler] = get_euler_pos()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_euler - a 3x1 array containing the Roll, Pitch and Yaw Euler angles (in SI units)
           
            vec_euler = obj.vec_euler;
            
        end
        
        function set_vec_euler(obj, vec_euler)
            % SET_VEC_EULER - Sets the vehicle orientation
            %
            % Syntax:  [] = set_vec_euler(vec_euler)
            %
            % Inputs:
            %    vec_euler - a 3x1 array containing the Roll, Pitch and Yaw Euler angles (in SI units)
            %
            % Outputs:
            %    (none)
            
            obj.vec_euler = vec_euler;
            
        end
        
        function vec_vel_linear_body = get_vec_vel_linear_body(obj)
            % GET_VEC_VEL_LINEAR_BODY - Returns the vehicle linear velocity, in body-frame
            %
            % Syntax:  [vec_vel_linear_body] = get_vec_vel_linear_body()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_vel_linear_body - a 3x1 array containing the he body-frame velocity (in SI units)
           
            vec_vel_linear_body = obj.vec_vel_linear_body;
            
        end
        
        function set_vec_vel_linear_body(obj, vec_vel_linear_body)
            % SET_VEC_VEL_LINEAR_BODY - Sets the vehicle linear velocity, in body-frame
            %
            % Syntax:  [] = set_vec_vel_linear_body(vec_vel_linear_body)
            %
            % Inputs:
            %    vec_vel_linear_body - a 3x1 array containing the body-frame velocity (in SI units)
            %
            % Outputs:
            %    (none)
            
            obj.vec_vel_linear_body = vec_vel_linear_body;
            
        end
        
        function vec_vel_angular_body = get_vec_vel_angular_body(obj)
            % GET_VEC_VEL_ANGULAR_BODY - Returns the vehicle angular velocity, in body-frame
            %
            % Syntax:  [vec_vel_angular_body] = get_vec_vel_angular_body()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_vel_angular_body - a 3x1 array containing the angular body-frame velocity (in SI units)
           
            vec_vel_angular_body = obj.vec_vel_angular_body;
            
        end
        
        function set_vec_vel_angular_body(obj, vec_vel_angular_body)
            % SET_VEC_VEL_ANGULAR_BODY - Sets the vehicle angular velocity, in body-frame
            %
            % Syntax:  [] = set_vec_vel_angular_body(vec_vel_angular_body)
            %
            % Inputs:
            %    vec_vel_angular_body - a 3x1 array containing the angular body-frame velocity (in SI units)
            %
            % Outputs:
            %    (none)
            
            obj.vec_vel_angular_body = vec_vel_angular_body;
            
        end
        
        function states = serialize(obj)
            % SERIALIZE Pack the vehicle state into a Nx1 vector
            % Take note of the state variables order
            %
            % Syntax:  [states] = serialize()
            %
            % Inputs:
            %   (none)
            %
            % Outputs:
            %    states - a 12x1 vector containing the NED position, Euler-angle orientation, body-frame linear velocity and
            %    body-frame angular velocity

            states = zeros(obj.num_states,1);
            
            states(1:3) = obj.get_vec_pos();
            states(4:6) = obj.get_vec_euler();
            states(7:9) = obj.get_vec_vel_linear_body();
            states(10:12) = obj.get_vec_vel_angular_body();
        end
        
    end
    
end

