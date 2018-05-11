classdef Gravity < handle
    %GRAVITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        g_0;
        vec_force_body;
        vec_torque_body;
    end
    
    methods
        
        function obj = Gravity(sim_options)
            obj.g_0 = sim_options.gravity.g_0;
            obj.vec_force_body = zeros(3,1);
            obj.vec_torque_body = zeros(3,1);
        end
        
        function calc_gravity(obj, vehicle)
            
            mass = vehicle.inertial.mass;
            
            vec_force_inertial = zeros(3,1);            
            vec_force_inertial(3) = obj.g_0*mass;
            
            obj.vec_force_body = vehicle.R_eb()*vec_force_inertial;
            
        end
        
        function vec_force_body = get_force_body(obj)
            
            vec_force_body = obj.vec_force_body;
            
        end
        
    end
    
end

