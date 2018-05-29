function [ control_vector ] = static_ouptut( vehicle_state, sim_time, static_control )
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
%
% See also: 

% Created at 2018/05/10 by George Zogopoulos-Papaliakos
% Last edit at 2018/05/29 by George Zogopoulos-Papaliakos

persistent ctrl_state

if isempty(ctrl_state)
    ctrl_state = zeros(4,1);
end

% Unused vehicle state, presented for template purposes
vec_pos = vehicle_state.get_vec_pos();
vec_euler = vehicle_state.get_vec_euler();
vec_vel_linear_body = vehicle_state.get_vec_vel_linear_body();
vec_vel_angular_body = vehicle_state.get_vec_vel_angular_body();

% Unused simulation time, presented for template purposes
t = sim_time;

if nargin>2
    ctrl_state = static_control;
end

control_vector = ctrl_state;

end

