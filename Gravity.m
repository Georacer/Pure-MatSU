classdef Gravity < handle
% GRAVITY Class containing the gravity model
%
% Other m-files required: simulation_options, Vehicle
% MAT-files required: none
%
% See also: simulation_options, Vehicle

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        g_0; % Nominal gravity value (in SI units)
        vec_force_body; % Gravity force in body-frame (in SI units)
        vec_torque_body; % Gravity torque in body-frame (in SI units), set to 0
    end
    
    methods
        
        function obj = Gravity(sim_options)
            % GRAVITY Class constructor
            %
            % Syntax:  [obj] = Gravity(sim_options)
            %
            % Inputs:
            %    sim_options - A struct output from the simulation_options function
            %
            % Outputs:
            %    obj - Class instance
            
            obj.g_0 = sim_options.gravity.g_0;
            obj.vec_force_body = zeros(3,1);
            obj.vec_torque_body = zeros(3,1);
        end
        
        function calc_gravity(obj, vehicle)
            % CALC_GRAVITY Perform the gravity model calculation
            % Utilizes the passed vehicle instance to access its state
            %
            % Syntax:  [] = calc_aerodynamics(vehicle)
            %
            % Inputs:
            %    vehicle - A Vehicle instance
            %
            % Outputs:
            %    (none)
            
            mass = vehicle.inertial.mass;
            
            vec_force_inertial = zeros(3,1);            
            vec_force_inertial(3) = obj.g_0*mass;
            
            obj.vec_force_body = vehicle.R_eb()*vec_force_inertial;
            
        end
        
        function vec_force_body = get_force_body(obj)
            % GET_FORCE_BODY Accessor for the gravity force
            %
            % Syntax:  [vec_force_body] = get_force_body()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    vec_force_body - a 3x1 vector containing the gravity force in body-frame (in SI units)
            
            vec_force_body = obj.vec_force_body;
            
        end
        
    end
    
end

