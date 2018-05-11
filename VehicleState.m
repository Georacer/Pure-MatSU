classdef VehicleState < handle
    %VEHICLESTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        vec_pos;
        vec_euler;
        vec_vel_linear_body;
        vec_vel_angular_body;
        
    end
    
    methods
        
        function obj = VehicleState()
           
            obj.vec_pos = zeros(3,1);
            obj.vec_euler = zeros(3,1);
            obj.vec_vel_linear_body = zeros(3,1);
            obj.vec_vel_angular_body = zeros(3,1);
            
        end
        
        function initialize(obj, sim_options)
           
            obj.vec_pos = sim_options.init.vec_pos;
            obj.vec_euler = sim_options.init.vec_euler;
            obj.vec_vel_linear_body = sim_options.init.vec_vel_linear_body;
            obj.vec_vel_angular_body = sim_options.init.vec_vel_angular_body;
            
        end
        
        function set_state(obj, external_state)
            
            obj.set_vec_pos(external_state.get_vec_pos());
            obj.set_vec_euler(external_state.get_vec_euler());
            obj.set_vec_vel_linear_body(external_state.get_vec_vel_linear_body());
            obj.set_vec_vel_angular_body(external_state.get_vec_vel_angular_body());
            
        end
        
        function vec_pos = get_vec_pos(obj)
            
            vec_pos = obj.vec_pos;
            
        end
        
        function set_vec_pos(obj, vec_pos)
            
            obj.vec_pos = vec_pos;
            
        end
        
        function vec_euler = get_vec_euler(obj)
           
            vec_euler = obj.vec_euler;
            
        end
        
        function set_vec_euler(obj, vec_euler)
            
            obj.vec_euler = vec_euler;
            
        end
        
        function vec_vel_linear_body = get_vec_vel_linear_body(obj)
           
            vec_vel_linear_body = obj.vec_vel_linear_body;
            
        end
        
        function set_vec_vel_linear_body(obj, vec_vel_linear_body)
            
            obj.vec_vel_linear_body = vec_vel_linear_body;
            
        end
        
        function vec_vel_angular_body = get_vec_vel_angular_body(obj)
           
            vec_vel_angular_body = obj.vec_vel_angular_body;
            
        end
        
        function set_vec_vel_angular_body(obj, vec_vel_angular_body)
            
            obj.vec_vel_angular_body = vec_vel_angular_body;
            
        end
        
    end
    
end

