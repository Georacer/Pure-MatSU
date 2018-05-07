function [ parameters ] = skywalker_2013( )
%SKYWALKER Summary of this function goes here
%   Detailed explanation goes here

%% Inertial characteristics
parameters.inertial.mass = 2.0; % in kg
parameters.inertial.j_x = 0.8244;
parameters.inertial.j_y = 1.135;
parameters.inertial.j_z = 1.759;
parameters.inertial.j_xz = 0.1204;


%% Aerodynamic characteristics
parameters.aerodynamics.aerodynamicsType = 1;

parameters.aerodynamics.s = 0.45; % in m^2
parameters.aerodynamics.b = 1.88; % in m
parameters.aerodynamics.c = 0.24; % in m

parameters.aerodynamics.c_lift_0 = 0.56;
parameters.aerodynamics.c_lift_deltae = 0.0;
parameters.aerodynamics.c_lift_a = 6.9;
parameters.aerodynamics.c_lift_q = 0;
parameters.aerodynamics.mcoeff = 50;
parameters.aerodynamics.oswald = 0.9;
parameters.aerodynamics.alpha_stall = 0.4712; % in rads
parameters.aerodynamics.c_drag_q = 0;
parameters.aerodynamics.c_drag_deltae = 0.0;
parameters.aerodynamics.c_drag_p = 0.0;
parameters.aerodynamics.c_y_0 = 0;
parameters.aerodynamics.c_y_b = -0.98;
parameters.aerodynamics.c_y_p = 0;
parameters.aerodynamics.c_y_r = 0;
parameters.aerodynamics.c_y_deltaa = 0;
parameters.aerodynamics.c_y_deltar = -0.2; %opposite sign than c_n_deltar

parameters.aerodynamics.c_l_0 = 0;
parameters.aerodynamics.c_l_p = -1.0;
parameters.aerodynamics.c_l_b = -0.12;
parameters.aerodynamics.c_l_r = 0.14;
parameters.aerodynamics.c_l_deltaa = 0.25;
parameters.aerodynamics.c_l_deltar = -0.037;
parameters.aerodynamics.c_m_0 = 0.045;
parameters.aerodynamics.c_m_a = -0.7;
parameters.aerodynamics.c_m_q = -20;
parameters.aerodynamics.c_m_deltae = 1.0;
parameters.aerodynamics.c_n_0 = 0;
parameters.aerodynamics.c_n_b = 0.25;
parameters.aerodynamics.c_n_p = 0.022;
parameters.aerodynamics.c_n_r = -1;
parameters.aerodynamics.c_n_deltaa = 0.00;
parameters.aerodynamics.c_n_deltar = 0.1; %opposite sign than c_y_deltar

parameters.aerodynamics.deltaa_max = 0.3491; % in rads
parameters.aerodynamics.deltae_max = 0.3491; % in rads
parameters.aerodynamics.deltar_max = 0.3491; % in rads

end

