classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rho;
        rho_0;
        rho_fh;
        
        wind_static = zeros(3,1);
    end
    
    methods
        
        function obj = Environment(sim_options)
            
            obj.rho_0 = sim_options.environment.rho_0;
            
            model_type = sim_options.environment.model_type;
            
            if model_type == 1
                obj.rho_fh = @obj.rho_mdl_1;
            else
                error("unknown model type");
            end
            
            obj.wind_static = sim_options.environment.wind;
            
        end
        
        function calc_state(obj, vehicle)
            
            % Calculate rho
            obj.rho = obj.rho_fh(vehicle);
            
        end
        
        function rho = rho_mdl_1(obj, vehicle)
            % Constant rho
            rho = obj.rho_0;
            
        end
        
        function rho = get_rho(obj)
            % Rho accessor
            rho = obj.rho;
        end
        
        function wind_ned = get_wind_ned(obj, vehicle)
            
            wind_ned = obj.wind_static;
            
        end
        
    end
    
end

