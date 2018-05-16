classdef Environment < handle
% ENVIRONMENT Environment class
% Implements a environment model, including air density and wind
%
% Other m-files required: Vehicle, simulation_options
% MAT-files required: none
%
% See also: Vehicle, simulation_options

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos
    
    properties
        rho; % Air density
        rho_0; % Nominal air density
        rho_fh; % function handle to air density calculator
        
        wind_static = zeros(3,1); % Constant wind vector
    end
    
    methods
        
        function obj = Environment(sim_options)
            % ENVIRONMENT Class constructor
            %
            % Syntax:  [obj] = Environment(sim_options)
            %
            % Inputs:
            %    model - Struct output from simulation_options function
            %
            % Outputs:
            %    obj - Class instance
            
            obj.rho_0 = sim_options.environment.rho_0;
            
            model_type = sim_options.environment.model_type;
            
            if model_type == 1
                obj.rho_fh = @obj.rho_mdl_1;
            else
                error('unknown model type');
            end
            
            obj.wind_static = sim_options.environment.wind;
            
        end
        
        function calc_state(obj, vehicle)
            % CALC_STATE Calculate the environmental quantities, given a Vehicle instance
            % Currently only updates the air density value
            %
            % Syntax:  [] = calc_state(vehicle_state)
            %
            % Inputs:
            %    vehicle - Vehicle object
            %
            % Outputs:
            %    (none)
            
            % Calculate rho
            obj.rho = obj.rho_fh(vehicle);
            
        end
        
        function rho = rho_mdl_1(obj, vehicle)
            % RHO_MDL_1 Static air density model
            % Returns the initialization air density value
            %
            % Syntax:  [] = rho_mdl_1(vehicle)
            %
            % Inputs:
            %    vehicle - Vehicle object
            %
            % Outputs:
            %    rho - Air density (in SI units)
            
            % Constant rho
            rho = obj.rho_0;
            
        end
        
        function rho = get_rho(obj)
            % GET_RHO Air density accessor
            % Returns the calculated air density value
            %
            % Syntax:  [] = rho_mdl_1()
            %
            % Inputs:
            %    (none)
            %
            % Outputs:
            %    rho - Air density (in SI units)
            
            rho = obj.rho;
        end
        
        function wind_ned = get_wind_ned(obj, vehicle)
            % GET_WIND_NED Wind vector accessor
            % Returns calculated wind vector at the vehicle location, in the NED frame
            % Currently returns only the static wind initialization value
            %
            % Syntax:  [] = get_wind_ned(vehicle)
            %
            % Inputs:
            %    vehicle - Vehicle object
            %
            % Outputs:
            %    wind_ned - Wind vector in NED coordinates (in SI units)
            
            wind_ned = obj.wind_static;
            
        end
        
    end
    
end

