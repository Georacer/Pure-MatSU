function plot_forces(gravity, propulsion, aerodynamics, t, initialize)
%
% modified 12/11/2009 - RB

    % define persistent variables 
    persistent gx_handle
    persistent gy_handle
    persistent gz_handle
    persistent g_handle
    persistent ax_handle
    persistent ay_handle
    persistent az_handle
    persistent a_handle
    persistent px_handle
    persistent py_handle
    persistent pz_handle
    persistent p_handle
    
    vec_gravity = gravity.get_force_body();
    gx=vec_gravity(1);
    gy=vec_gravity(2);
    gz=vec_gravity(3);
    g=norm(vec_gravity);
    
    vec_aerodynamics = aerodynamics.get_force_body();
    ax=vec_aerodynamics(1);
    ay=vec_aerodynamics(2);
    az=vec_aerodynamics(3);
    a=norm(vec_aerodynamics);
    
    vec_propulsion = propulsion.get_force_body();
    px=vec_propulsion(1);
    py=vec_propulsion(2);
    pz=vec_propulsion(3);
    p=norm(vec_propulsion);

  % first time function is called, initialize plot and persistent vars
    if initialize
        figure(), clf
        plotLines=6;

        subplot(plotLines,2,1)
        hold on
        gx_handle = graphForce(t, gx, 'gx', []);
        
        subplot(plotLines,2,2)
        hold on
        gy_handle = graphForce(t,gy, 'gy', []);

        subplot(plotLines,2,3)
        hold on
        gz_handle = graphForce(t, gz, 'gz', []);

        subplot(plotLines,2,4)
        hold on
        g_handle = graphForce(t,g, 'G', []);

        subplot(plotLines,2,5)
        hold on
        ax_handle = graphForce(t, ax, 'ax', []);

        subplot(plotLines,2,6)
        hold on
        ay_handle = graphForce(t, ay, 'ay', []);

        subplot(plotLines,2,7)
        hold on
        az_handle = graphForce(t, az, 'az', []);
        
        subplot(plotLines,2,8)
        hold on
        a_handle = graphForce(t, a, 'A', []);
        
        subplot(plotLines,2,9)
        hold on
        px_handle = graphForce(t, px, 'px', []);

        subplot(plotLines,2,10)
        hold on
        py_handle = graphForce(t, py, 'py', []);
        
        subplot(plotLines,2,11)
        hold on
        pz_handle = graphForce(t, pz, 'pz', []);
        
        subplot(plotLines,2,12)
        hold on
        p_handle = graphForce(t, p, 'p', []);
        
    % at every other time step, redraw state variables
    else 
       graphForce(t, gx, 'gx', gx_handle);
       graphForce(t, gy, 'gy', gy_handle);
       graphForce(t, gz, 'gz', gz_handle);
       graphForce(t, g, 'G', g_handle);
       graphForce(t, ax, 'ax', ax_handle);
       graphForce(t, ay, 'ay', ay_handle);
       graphForce(t, az, 'az', az_handle);
       graphForce(t, a, 'A', a_handle);
       graphForce(t, px, 'px', px_handle);
       graphForce(t, py, 'py', py_handle);
       graphForce(t, pz, 'psz', pz_handle);
       graphForce(t, p, 'P', p_handle);   
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graphForce(t, f, lab, handle)
  
  if isempty(handle)
    handle   = plot(t,f,'b');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),f]);
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

