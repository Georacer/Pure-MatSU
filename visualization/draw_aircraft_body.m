%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function plot_handle = draw_aircraft_body(vehicle, graphic, plot_handle)

  V_body = graphic.V;
  F = graphic.F;
  patch_colors = graphic.patch_colors;
  
  % Translate vertices
  V_ned = (vehicle.R_be()*V_body' + repmat(vehicle.state.get_vec_pos, 1, size(V_body,1)))';
  
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V_xyz = V_ned*R;
  
  if isempty(plot_handle)
  plot_handle = patch('Vertices', V_xyz, 'Faces', F,...
                 'FaceVertexCData',patch_colors,...
                 'FaceColor','flat');
  else
    set(plot_handle,'Vertices',V_xyz,'Faces',F);
    drawnow
  end
  
end