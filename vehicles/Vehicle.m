classdef Vehicle < handle
% VEHICLE General vehicle class
% Aggregates submodels and provides common functionality
%
% Other m-files required: VehicleState, Propulsion, Aerodynamics, Environment
% MAT-files required: none
%
% See also: VehicleState, Propulsion, Aerodynamics

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        inertial; % inertial parameters
        state; % pointer to VehicleState
        propulsion; % propulsion parameters
        aerodynamics; % aerodynamics parameters
        graphic; % Graphic representation of the vehicle, for drawing
    end
    
    methods
        
        function obj = Vehicle(model_name)
            % VEHICLE Class constructor
            %
            % Syntax:  [obj] = Vehicle(model)
            %
            % Inputs:
            %    model_name - Name of model. Must be the name of a model function in the vehicles folder e.g.
            %    "skywalker_2013".
            %
            % Outputs:
            %    obj - Class instance
            
            
            eval(sprintf("model = %s();", model_name));
            
            obj.state = VehicleState();
            
            obj.inertial = model.inertial;
            
            obj.propulsion = model.propulsion;
            
            obj.aerodynamics = model.aerodynamics;
            
            % Set graphic
            graphic_name = model.graphic;
            eval(sprintf("obj.graphic = %s();", graphic_name));
            
        end
        
        function set_state(obj, vehicle_state)
            % SET_STATE Copy an external vehicle_state to own member state
            %
            % Syntax:  [] = set_state(vehicle_state)
            %
            % Inputs:
            %    vehicle_state - VehicleState object
            %
            % Outputs:
            %    (none)
            
            obj.state.set_state(vehicle_state);
            
        end
        
        function state = get_state(obj)
            % GET_STATE - Accessor for the state member
            %
            % Syntax:  [state] = get_state()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    state - The internal VehicleState object
            
            state = obj.state;
            
        end
        
        function airdata = get_airdata(obj, environment)
            % GET_AIRDATA - Calculate the air data triplet (airspeed, AoA, AoS)
            % Requires an external Environment object to consult the wind model
            %
            % Syntax:  [airdata] = get_airdata(environment)
            %
            % Inputs:
            %    environment - An Environment instance
            %
            % Outputs:
            %    airdata - A 3x1 array containing airspeed, angle-of-attack and angle-of-sideslip (all in SI units)
            
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
            % R_EB Return the Earth-frame to Body-frame rotation matrix
            % Uses the internal state to calculate it
            %
            % Syntax:  [R] = R_eb( )
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    R - The rotation matrix
            
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
            % R_BE Return the Body-frame to Earth-frame rotation matrix
            %
            % Syntax:  [R] = R_be()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    R - The rotation matrix
           
            R = obj.R_eb()';
            
        end
        
    end
    
end

