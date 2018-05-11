function graphic = airplane_1()
% Simple airplane 3D Model
% Based on vertices and patches
% Author: George Zogopoulos-Papaliakos
% Date: 2018/05/11

%Geometrical Quantities Definition
fuse_h=0.1;
fuse_l1=0.3;
fuse_l2=-0.1;
fuse_l3=0.8;
fuse_w=0.1;
wing_l=0.3;
wing_w=1.2;
tailwing_l=0.2;
tailwing_w=0.5;
tail_h=0.2;

noVertices=16;
% Define the vertices (physical location of vertices)
V = [...
    fuse_l1    0           0;...           % point 1
    fuse_l2    fuse_w/2    -fuse_h/2;...   % point 2
    fuse_l2    -fuse_w/2   -fuse_h/2;...   % point 3
    fuse_l2    -fuse_w/2   fuse_h/2;...    % point 4
    fuse_l2    fuse_w/2    fuse_h/2;...    % point 5
    -fuse_l3   0           0;...           % point 6
    0          wing_w/2    0;...           % point 7
    -wing_l    wing_w/2    0;...           % point 8
    -wing_l    -wing_w/2   0;...           % point 9
    0          -wing_w/2   0;...           % point 10
    -fuse_l3+tailwing_l    tailwing_w/2    0;...       % point 11
    -fuse_l3   tailwing_w/2    0;...       % point 12
    -fuse_l3   -tailwing_w/2   0;...       % point 13
    -fuse_l3+tailwing_l    -tailwing_w/2   0;...       % point 14
    -fuse_l3+tailwing_l    0               0;...       % point 15
    -fuse_l3   0           -tail_h;...     % point 16
    ];

% place the center of the model on the aerodynamic center
V=V+[wing_l/4*ones(noVertices,1) zeros(noVertices,1) zeros(noVertices,1)];

% define faces as a list of vertices numbered above
F = [...
    1,2,3,3;...
    1,4,5,5;...
    1,3,4,4;...
    1,2,5,5;...
    2,3,6,6;...
    3,4,6,6;...
    4,5,6,6;...
    2,5,6,6;...
    7,8,9,10;...
    11,12,13,14;...
    6,15,16,16;...
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];
myblack= [0, 0, 0];

patch_colors = [...
    myred;
    myblue;
    myyellow;
    myyellow;
    myred;
    myyellow;
    myblue;
    myyellow;
    myyellow;
    myyellow
    myred;
    ];

graphic.V = V;
graphic.F = F;
graphic.patch_colors = patch_colors;

end