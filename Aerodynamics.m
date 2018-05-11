classdef Aerodynamics < handle
    %AERODYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vec_force_body;
        vec_torque_body;
        
        aerodynamics_mdl_fh;
    end
    
    methods
        
        function obj = Aerodynamics(vehicle)
           
            obj.vec_force_body = zeros(3,1);
            obj.vec_torque_body = zeros(3,1);
            
            model_type = vehicle.aerodynamics.model_type;
            
            if model_type == 1
                obj.aerodynamics_mdl_fh = @obj.aerodynamics_mdl_1;
            else
                error("unknown model type");
            end
            
        end
        
        function aerodynamics_mdl_1(obj, vehicle, environment, ctrl_input)
            % Standard model linear in the parameters
            
            % Read parameters
            s = vehicle.aerodynamics.s;
            b = vehicle.aerodynamics.b;
            c = vehicle.aerodynamics.c;
            c_drag_deltae = vehicle.aerodynamics.c_drag_deltae;
            c_drag_q = vehicle.aerodynamics.c_drag_q;
            c_lift_deltae = vehicle.aerodynamics.c_lift_deltae;  
            c_lift_q = vehicle.aerodynamics.c_lift_q;
            c_y_0 = vehicle.aerodynamics.c_y_0;
            c_y_b = vehicle.aerodynamics.c_y_b;
            c_y_p = vehicle.aerodynamics.c_y_p;
            c_y_r = vehicle.aerodynamics.c_y_r;
            c_y_deltaa = vehicle.aerodynamics.c_y_deltaa;
            c_y_deltar = vehicle.aerodynamics.c_y_deltar;
            c_l_0 = vehicle.aerodynamics.c_l_0;
            c_l_b = vehicle.aerodynamics.c_l_b;
            c_l_p = vehicle.aerodynamics.c_l_p;
            c_l_r = vehicle.aerodynamics.c_l_r;
            c_l_deltaa = vehicle.aerodynamics.c_l_deltaa;
            c_l_deltar = vehicle.aerodynamics.c_l_deltar;
            c_m_0 = vehicle.aerodynamics.c_m_0;
            c_m_a = vehicle.aerodynamics.c_m_a;
            c_m_q = vehicle.aerodynamics.c_m_q;
            c_m_deltae = vehicle.aerodynamics.c_m_deltae;
            c_n_0 = vehicle.aerodynamics.c_n_0;
            c_n_b = vehicle.aerodynamics.c_n_b;
            c_n_p = vehicle.aerodynamics.c_n_p;
            c_n_r = vehicle.aerodynamics.c_n_r;            
            c_n_deltaa = vehicle.aerodynamics.c_n_deltaa;
            c_n_deltar = vehicle.aerodynamics.c_n_deltar;
            
            % Read inputs
            aileron = ctrl_input(1)*vehicle.aerodynamics.deltaa_max; % in rads
            elevator = ctrl_input(2)*vehicle.aerodynamics.deltae_max; % in rads
            rudder = ctrl_input(4)*vehicle.aerodynamics.deltar_max; % in rads
            
            % Read angular velocities
            vehicle_state = vehicle.get_state;
            vec_vel_angular_body = vehicle_state.get_vec_vel_angular_body();
            p = vec_vel_angular_body(1);
            q = vec_vel_angular_body(2);
            r = vec_vel_angular_body(3);
            
            % Read environment data
            rho = environment.get_rho();
            airdata = vehicle.get_airdata(environment);
            airspeed = airdata(1);
            alpha = airdata(2);
            beta = airdata(3);
            
            % Calculate force
            
            % Calculate lift and drag alpha-coefficients
            c_lift_a = lift_coeff(vehicle, alpha);
            c_drag_a = drag_coeff(vehicle, alpha);
            
            % Convert coefficients to the body frame
            c_x_a = -c_drag_a*cos(alpha) + c_lift_a*sin(alpha);
            c_x_q = -c_drag_q*cos(alpha) + c_lift_q*sin(alpha);
            c_z_a = -c_drag_a*sin(alpha) - c_lift_a*cos(alpha);
            c_z_q = -c_drag_q*sin(alpha) - c_lift_q*cos(alpha);
            
            q_bar = 0.5*rho*airspeed^2*s;
            
            if (airspeed==0)
                obj.vec_force_body = zeros(3,1);
            else
                obj.vec_force_body(1) = q_bar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*abs(elevator) + c_lift_deltae*sin(alpha)*elevator);
                obj.vec_force_body(2) = q_bar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*aileron + c_y_deltar*rudder);
                obj.vec_force_body(3) = q_bar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*abs(elevator) - c_lift_deltae*cos(alpha)*elevator);
            end
            
            % Calculate torque
            
            if (airspeed==0)
                obj.vec_torque_body = zeros(3,1);
            else
                obj.vec_torque_body(1) = q_bar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*airspeed) + c_l_r*b*r/(2*airspeed) + c_l_deltaa*aileron + c_l_deltar*rudder);
                obj.vec_torque_body(2) = q_bar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*airspeed) + c_m_deltae*elevator);
                obj.vec_torque_body(3) = q_bar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*airspeed) + c_n_r*b*r/(2*airspeed) + c_n_deltaa*aileron + c_n_deltar*rudder);
            end
            
           
            function cL_a = lift_coeff(vehicle, alpha)
                % Caclulate lift coefficient
                
                % Read parameters
                M = vehicle.aerodynamics.mcoeff;
                alpha_0 = vehicle.aerodynamics.alpha_stall;
                c_lift_0 = vehicle.aerodynamics.c_lift_0;
                c_lift_a0 = vehicle.aerodynamics.c_lift_a;
                
                sigmoid = (1 + exp(-M*(alpha-alpha_0)) + exp(M*(alpha+alpha_0)) ) / (1 + exp(-M*(alpha-alpha_0))) / (1 + exp(M*(alpha+alpha_0)));
                linear = (1 - sigmoid)*(c_lift_0 + c_lift_a0*alpha); % Lift at small AoA
                flat_plate = sigmoid*(2*sign(alpha)*(sin(alpha))^2*cos(alpha)); % Lift beyond stall
                
                cL_a = linear + flat_plate;
                
            end
            
            function cD_a = drag_coeff(vehicle, alpha)
                % Calculate drag coefficient
                
                % Read parameters
                b1 = vehicle.aerodynamics.b;
                s1 = vehicle.aerodynamics.s;
                c_drag_p = vehicle.aerodynamics.c_drag_p;
                c_lift_0 = vehicle.aerodynamics.c_lift_0;
                c_lift_a0 = vehicle.aerodynamics.c_lift_a;
                oswald = vehicle.aerodynamics.oswald;
                
                % Calculate quantities
                AR = b1^2/s1;
                cD_a = c_drag_p + (c_lift_0 + c_lift_a0*alpha)^2/(pi*oswald*AR);
                
            end
            
        end
        
        function calc_aerodynamics(obj, vehicle, environment, ctrl_input)
            
            obj.aerodynamics_mdl_fh(vehicle, environment, ctrl_input);
            
        end
        
        function vec_force_body = get_force_body(obj)
            
            vec_force_body = obj.vec_force_body;
        end
        
        function vec_torque_body = get_torque_body(obj)
            
            vec_torque_body = obj.vec_torque_body;
        end
        
    end
    
end

