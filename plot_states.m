function plot_states(vehicle, t, initialize)
%
% modified 12/11/2009 - RB

    % process inputs to function
    vec_pos = vehicle.state.get_vec_pos();
    pn          = vec_pos(1);             % North position (meters)
    pe          = vec_pos(2);             % East position (meters)
    pd           = vec_pos(3);            % altitude (meters)
    
    vec_euler = vehicle.state.get_vec_euler();
    phi         = wrapTo180(180/pi*vec_euler(1));      % roll angle (degrees)   
    theta       = wrapTo180(180/pi*vec_euler(2));      % pitch angle (degrees)
    psi         = wrapTo180(180/pi*vec_euler(3));             % yaw angle (degrees)    
    
    vec_vel_linear_body = vehicle.state.get_vec_vel_linear_body();
    u           = vec_vel_linear_body(1);             % body velocity along x-axis (meters/s)
    v           = vec_vel_linear_body(2);             % body velocity along y-axis (meters/s)
    w           = vec_vel_linear_body(3);             % body velocity along z-axis (meters/s)
    
    vec_vel_angular_body = vehicle.state.get_vec_vel_angular_body();
    p           = 180/pi*vec_vel_angular_body(1);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*vec_vel_angular_body(2);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*vec_vel_angular_body(3);     % body angular rate along z-axis (degrees/s)
    
%     airdata = vehicle.get_airdata(environment);
%     Va          = airdata(1);            % airspeed (m/s)
%     alpha       = 180/pi*airdata(2);     % angle of attack (degrees)
%     beta        = 180/pi*airdata(3);     % side slip angle (degrees)
    
%     wn          = uu(16);            % wind in the North direction
%     we          = uu(17);            % wind in the East direction
%     wd          = uu(18);            % wind in the Down direction
%     pn_c        = uu(19);            % commanded North position (meters)
%     pe_c        = uu(20);            % commanded East position (meters)
%     h_c         = uu(21);            % commanded altitude (meters)
%     Va_c        = uu(22);            % commanded airspeed (meters/s)
%     alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
%     beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
%     phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
%     theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
%     chi_c       = 180/pi*uu(27);     % commanded course (degrees)
%     p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
%     q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
%     r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
%     pn_hat      = uu(31);            % estimated North position (meters)
%     pe_hat      = uu(32);            % estimated East position (meters)
%     h_hat       = uu(33);            % estimated altitude (meters)
%     Va_hat      = uu(34);            % estimated airspeed (meters/s)
%     alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
%     beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
%     phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)   
%     theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
%     p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
%     q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
%     r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
%     Vg_hat      = uu(43);            % estimated groundspeed
%     wn_hat      = uu(44);            % estimated North wind
%     we_hat      = uu(45);            % estimated East wind
%     psi_hat     = 180/pi*uu(46);     % estimated heading

%     delta_e     = 180/pi*uu(47);     % elevator angle (degrees)
%     delta_a     = 180/pi*uu(48);     % aileron angle (degrees)
%     delta_r     = 180/pi*uu(49);     % rudder angle (degrees)
%     delta_t     = uu(50);            % throttle setting (unitless)

    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent pd_handle
%     persistent Va_handle
%     persistent alpha_handle
%     persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
%     persistent delta_e_handle
%     persistent delta_a_handle
%     persistent delta_r_handle
%     persistent delta_t_handle
    

  % first time function is called, initialize plot and persistent vars
    if initialize
        num_lines = 5;
        figure(), clf

        subplot(num_lines,2,1)
        hold on
        pn_handle = graph_y(t, pn, [], 'pn', 'b');        
        subplot(num_lines,2,2)
        hold on
        pe_handle = graph_y(t, pe, [], 'pe', 'b');
        subplot(num_lines,2,3)
        hold on
        pd_handle = graph_y(t, pd, [], 'pd', 'b');
        
%         subplot(8,2,2)
%         hold on
%         Va_handle = graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', []);

%         subplot(8,2,4)
%         hold on
%         alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);

%         subplot(8,2,6)
%         hold on
%         beta_handle = graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', []);

        subplot(num_lines,2,4)
        hold on
        phi_handle = graph_y(t, phi, [], '\phi', 'b');
        set(gca,'YLim',[-180 180]);
        set(gca,'YTick',-180:90:180);        
        subplot(num_lines,2,5)
        hold on
        theta_handle = graph_y(t, theta, [], '\theta', 'b');
        set(gca,'YLim',[-180 180]);
        set(gca,'YTick',-180:90:180);        
        subplot(num_lines,2,6)
        hold on
        psi_handle = graph_y(t, psi, [], '\psi', 'b');
        set(gca,'YLim',[-180 180]);
        set(gca,'YTick',-180:90:180);
        
        subplot(num_lines,2,7)
        hold on
        p_handle = graph_y(t, p, [], 'p', 'b');        
        subplot(num_lines,2,8)
        hold on
        q_handle = graph_y(t, q, [], 'q', 'b');        
        subplot(num_lines,2,9)
        hold on
        r_handle = graph_y(t, r, [], 'r', 'b');
        
%         subplot(8,2,13)
%         hold on
%         delta_e_handle = graph_y(t, delta_e, [], 'b');
%         ylabel('\delta_e')
%         
%         subplot(8,2,14)
%         hold on
%         delta_a_handle = graph_y(t, delta_a, [], 'b');
%         ylabel('\delta_a')
% 
%         subplot(8,2,15)
%         hold on
%         delta_r_handle = graph_y(t, delta_r, [], 'b');
%         ylabel('\delta_r')
%         
%         subplot(8,2,16)
%         hold on
%         delta_t_handle = graph_y(t, delta_t, [], 'b');
%         ylabel('\delta_t')
        
    % at every other time step, redraw state variables
    else 
       graph_y(t, pn, pn_handle);
       graph_y(t, pe, pe_handle);
       graph_y(t, pd, pd_handle);
%        graph_y(t, Va, Va_hat, Va_c, 'V_a', Va_handle);
%        graph_y(t, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
%        graph_y(t, beta, beta_hat, beta_c, '\beta', beta_handle);
       graph_y(t, phi, phi_handle);
       graph_y(t, theta, theta_handle);
       graph_y(t, psi, psi_handle);
       graph_y(t, p, p_handle);
       graph_y(t, q, q_handle);
       graph_y(t, r, r_handle);
%        graph_y(t, delta_e, delta_e_handle);
%        graph_y(t, delta_a, delta_a_handle);
%        graph_y(t, delta_r, delta_r_handle);
%        graph_y(t, delta_t, delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with label mylabel
function handle = graph_y(t, y, handle, lab, color)
  
  if isempty(handle)
    handle    = plot(t,y,color);
    ylabel(lab);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with label mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab);
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle)
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low
      out = low;
  elseif in > high
      out = high;
  else
      out = in;
  end

% end sat  

