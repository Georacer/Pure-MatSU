function draw_aircraft(vehicle, graphic, initialize)
%Function to create the visualisation of the aircraft
%V is a vector of the model vertices
%F is a vector of the model edges
%patchcolors is a vector of the surface colors

    % define persistent variables 
    persistent spacecraft_handle;
    persistent axes_handle;
    persistent zoom;
    persistent follow;
    follow = true;
    persistent record;
    record = false;
    persistent XPath;
    persistent YPath;
    persistent ZPath;
    persistent line_axis;
    persistent figure_handle;
    persistent frame_counter;
    if initialize
        frame_counter = 1;
    end

    if record
        global mymov
    end

    % process inputs to function
    vec_pos = vehicle.state.get_vec_pos();
    pn = vec_pos(1); % inertial North position     
    pe = vec_pos(2); % inertial East position
    pd = vec_pos(3); % Down coordinate
    pz = -pd;

    % first time function is called, initialize plot and persistent vars
    if frame_counter==1
        figure_handle = figure();, clf
        axes_handle=gca;
        if follow
            zoom = 2;
        else
            zoom = 2;
        end
        spacecraft_handle = drawSpacecraftBody(vehicle,...
                                               graphic,...
                                               []);
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(0,70)  % set the view angle for figure
        grid on
        if follow
            set(axes_handle,'XLim',[-zoom+pe zoom+pe],'YLim',[-zoom+pn zoom+pn],'ZLim',[-zoom+pz zoom+pz]);
        else
            set(axes_handle,'XLim',[-zoom zoom],'YLim',[-zoom zoom],'ZLim',[-zoom zoom]);
        end
        hold on
        
        XPath = [pe];
        YPath = [pn];
        ZPath = [pz];
        line_axis = plot3(XPath,YPath,ZPath);
        
        axis equal
        
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(vehicle,...
                           graphic,...
                           spacecraft_handle);
        if follow
            set(axes_handle,'XLim',[-zoom+pe zoom+pe],'YLim',[-zoom+pn zoom+pn],'ZLim',[-zoom+pz zoom+pz]);
        else
            set(axes_handle,'XLim',[-zoom zoom],'YLim',[-zoom zoom],'ZLim',[-zoom zoom]);
        end
                
        XPath = [XPath pe];
        YPath = [YPath pn];
        ZPath = [ZPath pz];
        set(line_axis,'XData',XPath,'YData',YPath,'ZData',ZPath)
    end
    frame_counter = frame_counter + 1;
    
    if record
        mymov(frame_counter)=getframe(figure_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(vehicle,...
                                     graphic,...
                                     handle)

  V_body = graphic.V;
  F = graphic.F;
  patch_colors = graphic.patch_colors;
  
  % Translate vertices
%   V_ned = vehicle.R_be()*V_body + vehicle.state.get_vec_pos;
%   V_ned = V_body*vehicle.R_be() + repmat(vehicle.state.get_vec_pos', size(V_body,1), 1);
  V_ned = (vehicle.R_be()*V_body' + repmat(vehicle.state.get_vec_pos, 1, size(V_body,1)))';
  
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V_xyz = V_ned*R;
  
  if isempty(handle)
  handle = patch('Vertices', V_xyz, 'Faces', F,...
                 'FaceVertexCData',patch_colors,...
                 'FaceColor','flat');
  else
    set(handle,'Vertices',V_xyz,'Faces',F);
    drawnow
  end
  
end