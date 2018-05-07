classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rho_0;
        rho_fh;
    end
    
    methods
        
        function obj = Environment(sim_options)
            
            model_type = sim_options.environment.model_type;
            
            if model_type == 1
                
            else
                error("unknown model type"0;
            end
            
        end
        
        function rho = get_rho(obj, vehicle_state)
            rho = 
        end
        
        function state = get_state(obj, vehicle_state)
            
            
            
            
        end
        
    end
    
end

