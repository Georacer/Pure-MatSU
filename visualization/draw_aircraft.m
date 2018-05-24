function draw_aircraft(vehicle, initialize, varargin)
%Function to create the visualisation of the aircraft

    parser = inputParser;
    parser.addParameter('figure_handle', []);
    parser.parse(varargin{:});

    % define persistent variables 
    persistent aircraft_handle;
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
        if ~isempty(parser.Results.figure_handle)
            figure_handle = p.Results.figure_handle;
        else
            figure_handle = figure();
        end
        axes_handle=gca;
        if follow
            zoom = 2;
        else
            zoom = 2;
        end
        aircraft_handle = draw_aircraft_body(vehicle, []);
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
        draw_aircraft_body(vehicle, aircraft_handle);
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