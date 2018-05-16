function [ control_vector ] = zero_ouptut( vehicle_state )
% ZERO_OUTPUT Returns a constant zero control output
%
% Syntax:  [obj] = zero_ouptut([])
%
% Inputs:
%    vehicle_state - A VehicleState instance
%
% Outputs:
%    control_vector - A 4x1 array filled with zeros
%
% Other m-files required: 
% MAT-files required: none
%
% See also: 

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/16 by George Zogopoulos-Papaliakos

% Unused vehicle state, presented for template purposes
vec_pos = vehicle_state.get_vec_pos();
vec_euler = vehicle_state.get_vec_euler();
vec_vel_linear_body = vehicle_state.get_vec_vel_linear_body();
vec_vel_angular_body = vehicle_state.get_vec_vel_angular_body();

control_vector = zeros(4,1);


end

