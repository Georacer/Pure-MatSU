classdef Vehicle < handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        inertial;
        state;
        propulsion; % propulsion parameters
        aerodynamics; % aerodynamics parameters
    end
    
    methods
        
        function obj = Vehicle(model)
            % Constructor
            
            obj.state = VehicleState();
            
            obj.inertial = model.inertial;
            
            obj.propulsion = model.propulsion;
            
            obj.aerodynamics = model.aerodynamics;
        end
        
        function set_state(obj, vehicle_state)
            % Copy an external vehicle_state to own member state
            
            obj.state.set_state(vehicle_state);
            
        end
        
        function state = get_state(obj)
           
            state = obj.state;
            
        end
        
        function airdata = get_airdata(obj, environment)
           
            % Read wind
            wind_ned = environment.get_wind_ned(obj);
            % convert from Earth-frame to body-frame
            wind_body = obj.R_eb()*wind_ned;
            
            % Calc relative airspeed
            vel_linear_body = obj.state.get_vec_vel_linear_body();
            vel_linear_body_relative = vel_linear_body - wind_body;
            
            % Calc airspeed
            airspeed = norm(vel_linear_body_relative); % in m/s
            
            u_r = vel_linear_body_relative(1);
            v_r = vel_linear_body_relative(2);
            w_r = vel_linear_body_relative(3);
            
            % Calc alpha
            alpha = atan2(w_r,u_r); % in radians
            
            % Calc beta
            % in radians
            if (u_r==0)
                if (v_r==0)
                    beta = 0;
                else
                    beta = asin(v_r/abs(v_r));
                end
            else
                beta = atan2(v_r,u_r);
            end
            
            airdata = [airspeed; alpha; beta];
            
        end
        
        function R = R_eb(obj)
           % Return rotation matrix from earth frame to body frame
           
           R = zeros(3,3);
           
           vec_euler = obj.state.get_vec_euler();
           Phi = vec_euler(1);
           Theta = vec_euler(2);
           Psi = vec_euler(3);           
           
           sinPhi = sin(Phi);
           cosPhi = cos(Phi);
           sinTheta = sin(Theta);
           cosTheta = cos(Theta);
           sinPsi = sin(Psi);
           cosPsi = cos(Psi);

           R(1,1) = cosTheta * cosPsi;
           R(1,2) = cosTheta * sinPsi;
           R(1,3) = -sinTheta;
           R(2,1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
           R(2,2) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
           R(2,3) = sinPhi * cosTheta;
           R(3,1) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
           R(3,2) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
           R(3,3) = cosPhi * cosTheta;
           
        end
        
        function R = R_be(obj)
           
            R = obj.R_eb()';
            
        end
        
    end
    
end

