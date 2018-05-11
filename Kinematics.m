classdef Kinematics < handle
    %KINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vec_force_body;
        vec_torque_body;
        dt;
        state;
        vec_pos_dot;
        vec_euler_dot;
        vec_vel_linear_body_dot;
        vec_vel_angular_body_dot;
    end
    
    methods
        
        function obj = Kinematics(sim_options)
            
            obj.vec_force_body = zeros(3,1);
            obj.vec_torque_body = zeros(3,1);
            obj.state = VehicleState();
            obj.vec_pos_dot = zeros(3,1);
            obj.vec_euler_dot = zeros(3,1);
            obj.vec_vel_linear_body_dot = zeros(3,1);
            obj.vec_vel_angular_body_dot = zeros(3,1);
            obj.dt = sim_options.dt;
            
        end
        
        function set_wrench_body(obj, vec_force_body, vec_torque_body)
           
            obj.vec_force_body = vec_force_body;
            obj.vec_torque_body = vec_torque_body;
            
        end
        
        function calc_state_derivatives(obj, vehicle)
            
            % Setup the matrix of inertia
            J = zeros(3,3);
            J(1,1) = vehicle.inertial.j_x;
            J(2,2) = vehicle.inertial.j_y;
            J(3,3) = vehicle.inertial.j_z;
            J(1,3) = -vehicle.inertial.j_xz;
            J(3,1) = -vehicle.inertial.j_xz;
            % Calcualte its inverse
            J_i = J^(-1);
            
            % Read rotation matrix
            R_be = vehicle.R_be();
            
            % Calculate position derivative
            vec_vel_linear_body_prev = obj.state.get_vec_vel_linear_body();
            obj.vec_pos_dot = R_be*vec_vel_linear_body_prev;
            
            % Calculate velocity derivative
            vec_vel_angular_body_prev = obj.state.get_vec_vel_angular_body();
            vec_euler_prev = obj.state.get_vec_euler();
            phi = vec_euler_prev(1);
            theta = vec_euler_prev(2);
            psi = vec_euler_prev(3);
            E = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
                0 cos(phi) -sin(phi);...
                0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
            obj.vec_euler_dot = E*vec_vel_angular_body_prev;
            
            % Calculate linear velocity derivatives
            linear_acc = obj.vec_force_body/vehicle.inertial.mass;
            corriolis_acc = -cross(vec_vel_angular_body_prev, vec_vel_linear_body_prev);
            obj.vec_vel_linear_body_dot = linear_acc + corriolis_acc;
            
            % Calculate angular velocity derivatives
            obj.vec_vel_angular_body_dot = J_i*(obj.vec_torque_body - cross(vec_vel_angular_body_prev, (J*vec_vel_angular_body_prev)));
            
        end
        
        function answer = get_state_derivatives(obj)
            
            answer.vec_pos_dot = obj.vec_pos_dot;
            answer.vec_euler_dot = obj.vec_euler_dot;
            answer.vec_vel_linear_body_dot = obj.vec_vel_linear_body_dot;
            answer.vec_vel_angular_body_dot = obj.vec_vel_angular_body_dot;            
            
        end
        
        function integrate(obj)            
            
            obj.state.set_vec_pos(obj.state.get_vec_pos() + obj.vec_pos_dot*obj.dt);
            obj.state.set_vec_euler(obj.state.get_vec_euler() + obj.vec_euler_dot*obj.dt);
            obj.state.set_vec_vel_linear_body(obj.state.get_vec_vel_linear_body() + obj.vec_vel_linear_body_dot*obj.dt);
            obj.state.set_vec_vel_angular_body(obj.state.get_vec_vel_angular_body() + obj.vec_vel_angular_body_dot*obj.dt);
            
        end
        
        function set_state(obj, external_state)
            
            obj.state.set_state(external_state);
            
        end
        
        function state = get_state(obj)
            % Create a new vehicle state object, fill it and return it
           
            state = VehicleState();
            state.set_state(obj.state);
            
        end
        
        function write_state(obj, external_state)
            % Fill the external_state with own state object values
            
            external_state.set_vec_pos(obj.state.get_vec_pos());
            external_state.set_vec_euler(obj.state.get_vec_euler());
            external_state.set_vec_vel_linear_body(obj.state.get_vec_vel_linear_body());
            external_state.set_vec_vel_angular_body(obj.state.get_vec_vel_angular_body());   
            
        end
        
    end
    
end

