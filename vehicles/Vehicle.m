classdef Vehicle < handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mass; % in kg
        S; % in m^2   
        vec_pos;
        vec_euler;
        vec_vel_linear_body;
        vec_vel_angular_body;
    end
    
    methods
        
        function obj = Vehicle(model)
            % Constructor
            obj.mass = model.inertial.mass;
            obj.S = model.S;
            
            obj.vec_pos = model.
            obj.vec_euler = model.
            obj.vel_linear_body = model.
            obj.vel_angular_body = model.
            
        end
        
        function R = R_eb()
           % Return rotation matrix from earth frame to body frame
           
           R = zeros(3,3);
           
           sinPhi = sin(Phi);
           cosPhi = cos(Phi);
           sinTheta = sin(Theta);
           cosTheta = cos(Theta);
           sinPsi = sin(Psi);
           cosPsi = cos(Psi);

           R(1,1) = cosTheta * cosPsi;
           R(1,2) = cosTheta * sinPsi;
           R(1,3) = -sinTheta;
           R(2,1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
           R(2,2) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
           R(2,3) = sinPhi * cosTheta;
           R(3,1) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
           R(3,2) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
           R(3,3) = cosPhi * cosTheta;
           
        end
        
    end
    
end

