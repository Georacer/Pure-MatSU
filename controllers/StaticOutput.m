classdef StaticOutput < handle
% STATICOUTPUT Controller, which generates a constant output.
% Also serves as controller class template
%
% Other m-files required: VehicleState
% MAT-files required: none
%
% See also: VehicleState
%
% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/29 by George Zogopoulos-Papaliakos
    
    properties
        output_preset; % Container for static control output
    end
    
    methods
        
        function [obj] = StaticOutput(output_preset)
            % STATICOUTPUT Class constructor
            %
            % Syntax:  [obj] = StaticOutput(output_preset)
            %
            % Inputs:
            %    output_preset - A 4x1 array comprised of aileron input [-1,1], elevator input [-1,1], throttle input [0,1]
            %    and rudder input [-1,1]
            %
            % Outputs:
            %    obj - Class instance
            
            obj.output_preset = output_preset;
            
        end
        
        function [ control_vector ] = calc_output(obj, vehicle_state, sim_time)
            % STATIC_OUTPUT Returns a constant control output
            %
            % Syntax:  [obj] = static_ouptut([], [0; 0; 0.5; 0])
            %
            % Inputs:
            %    vehicle_state - A VehicleState instance
            %    sim_time - The current simulation timestamp (in seconds)
            %    static_control - A 4x1 array comprised of aileron input [-1,1], elevator input [-1,1], throttle input [0,1]
            %    and rudder input [-1,1]
            %
            % Outputs:
            %    control_vector - A 4x1 array returning the static control vector, as initialized by the static_control input
            %
            % Other m-files required:
            % MAT-files required: none
            
            % Unused vehicle state, presented for template purposes
            vec_pos = vehicle_state.get_vec_pos();
            vec_euler = vehicle_state.get_vec_euler();
            vec_vel_linear_body = vehicle_state.get_vec_vel_linear_body();
            vec_vel_angular_body = vehicle_state.get_vec_vel_angular_body();
            
            % Unused simulation time, presented for template purposes
            t = sim_time;
            
            % Generate control output
            control_vector = obj.output_preset;
            
        end
        
    end
    
end