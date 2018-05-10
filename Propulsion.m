classdef Propulsion < handle
    %PROPULSION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        vec_force_body;
        vec_torque_body;
        
        propulsion_mdl_fh;
        
    end
    
    methods
        
        function obj = Propulsion(vehicle)
           
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
            % Beard electric engine
            
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
            
            obj.propulsion_mdl_fh(vehicle, environment, ctrl_input);
            
        end
        
        function vec_force_body = get_force_body(obj)
            
            vec_force_body = obj.vec_force_body;
        end
        
        function vec_torque_body = get_torque_body(obj)
            
            vec_torque_body = obj.vec_torque_body;
        end
        
    end
    
end

