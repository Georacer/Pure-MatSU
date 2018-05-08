classdef Gravity < handle
    %GRAVITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        g_0;
    end
    
    methods
        
        function obj = Gravity(sim_options)
            obj.g_0 = sim_options.gravity.g_0;
        end
        
        function vec_force_body = get_gravity_force(obj, vehicle)
            
            mass = vehicle.inertial.mass;
            
            vec_force_inertial = zeros(3,1);
            
            vec_force_inertial(3) = obj.g_0*mass;
            
            vec_force_body = vehicle.R_eb()*vec_force_inertial;
            
        end
        
    end
    
end

