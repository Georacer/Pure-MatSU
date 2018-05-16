classdef Controller < handle
% CONTROLLER General class to provide controller API
% Aggregates submodels and provides common functionality
%
% Other m-files required: simulation_options, VehicleState
% MAT-files required: none
%
% See also: simulation_states, VehicleState
%
% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        controller_fh; % Function handle to the selected controller
    end
    
    methods
        
        function obj = Controller(sim_options)
            % CONTROLLER Class constructor
            %
            % Syntax:  [obj] = Controller(sim_options)
            %
            % Inputs:
            %    sim_options - Struct output from simulation_options
            %
            % Outputs:
            %    obj - Class instance
            
            controller_type = sim_options.controller.type;
            
            % Select controller type
            if controller_type == 0
                obj.controller_fh = @zero_ouptut;
            else
                error('unknown controller type');
            end
            
        end
        
        function control_vector = gen_control(obj, vehicle_state)
            % CONTROLLER Class constructor
            % Calls the selected controller function and delegates the controls calculation. Passes the vehicle state to
            % it.
            %
            % Syntax:  [obj] = Controller(vehicle_state)
            %
            % Inputs:
            %    vehicle_state - A VehicleState instance
            %
            % Outputs:
            %    control_vector - A 4x1 array comprised of aileron input [-1,1], elevator input [-1,1], throttle input [0,1]
            %    and rudder input [-1,1]
            
            % Call external controller to calculate controls
            control_vector = obj.controller_fh(vehicle_state);
            
        end
        
    end
    
end

