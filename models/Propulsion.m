classdef Propulsion < handle
% PROPULSIOn Class containing all propulsion calculations
%
% Other m-files required: Vehicle, Environment
% MAT-files required: none
%
% See also: Vehicle, Environment

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        
        vec_force_body; % body-frame propulsion force, in SI units
        vec_torque_body; % body-frame propulsion torque, in SI units
        
        propulsion_mdl_fh; % function handle to propulsion model
        
    end
    
    methods
        
        function obj = Propulsion(vehicle)
            % PROPULSION Class constructor
            %
            % Syntax:  [obj] = Propulsion(vehicle)
            %
            % Inputs:
            %    vehicle - A Vehicle instance
            %
            % Outputs:
            %    obj - Class instance
           
            obj.vec_force_body = zeros(3,1);
            obj.vec_torque_body = zeros(3,1);
            
            model_type = vehicle.propulsion.model_type;
            
            if model_type == 1
                obj.propulsion_mdl_fh = @obj.propulsion_mdl_1;
            else
                error("unknown model type");
            end
            
        end
        
        function propulsion_mdl_1(obj, vehicle, environment, ctrl_input)
            % PROPULSION_MDL_1 Simplistic electric motor model
            % Found in Beard and McLain's book "Small Unmanned Aircraft"
            %
            % Syntax:  [] = propulsion_mdl_1(vehicle, environment, ctrl_input)
            %
            % Inputs:
            %    vehicle - A Vehicle instance
            %    environment - An Environment instance
            %    ctrl_input - A 4x1 array comprised of aileron input [-1,1], elevator input [-1,1], throttle input [0,1]
            %    and rudder input [-1,1]
            %
            % Outputs:
            %    (none)
            
            % Read parameters
            s_prop = vehicle.propulsion.s_prop;
            c_prop = vehicle.propulsion.c_prop;
            k_motor = vehicle.propulsion.k_motor;
            k_t_p = vehicle.propulsion.k_t_p;
            k_omega = vehicle.propulsion.k_omega;
            
            % Read inputs
            throttle = ctrl_input(3);
            
            % Read environment data
            rho = environment.get_rho();
            airdata = vehicle.get_airdata(environment);
            airspeed = airdata(1);
            
            omega = throttle*k_omega; % in rad/s
            
            % Calculate force
            obj.vec_force_body(1) = 0.5*rho*s_prop*c_prop*((throttle*k_motor)^2 - airspeed^2);
            obj.vec_force_body(2) = 0;
            obj.vec_force_body(3) = 0;
            
            % Calculate torque
            obj.vec_torque_body(1) = sign(omega)*k_t_p*omega^2;
            obj.vec_torque_body(2) = 0;
            obj.vec_torque_body(3) = 0;
            
        end        
        
        function calc_propulsion(obj, vehicle, environment, ctrl_input)
            % CALC_PROPULSION Perform the propulsion calculation
            %
            % Syntax:  [] = calc_propulsion(vehicle, environment, ctrl_input)
            %
            % Inputs:
            %    vehicle - A Vehicle instance
            %    environment - An Environment instance
            %    ctrl_input - A 4x1 array comprised of aileron input [-1,1], elevator input [-1,1], throttle input [0,1]
            %    and rudder input [-1,1]
            %
            % Outputs:
            %    (none)
            
            obj.propulsion_mdl_fh(vehicle, environment, ctrl_input);
            
        end
        
        function vec_force_body = get_force_body(obj)
            % GET_FORCE_BODY Accessor for the propulsion force
            %
            % Syntax:  [vec_force_body] = get_force_body()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_force_body - a 3x1 vector containing the propulsion force in body-frame (in SI units)
            
            vec_force_body = obj.vec_force_body;
        end
        
        function vec_torque_body = get_torque_body(obj)
            % GET_FORCE_BODY Accessor for the propulsion torque
            %
            % Syntax:  [vec_force_body] = get_torque_body()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_torque_body - a 3x1 vector containing the propulsion torque in body-frame (in SI units)
            
            vec_torque_body = obj.vec_torque_body;
        end
        
    end
    
end

