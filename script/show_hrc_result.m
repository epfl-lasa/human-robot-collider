%% 3D Model Demo
% This is short demo that loads and renders a 3D model of a human femur. It
% showcases some of MATLAB's advanced graphics features, including lighting and
% specular reflectance.

% Copyright 2011 The MathWorks, Inc.


%% Load STL mesh
% Stereolithography (STL) files are a common format for storing mesh data. STL
% meshes are simply a collection of triangular faces. This type of model is very
% suitable for use with MATLAB's PATCH graphics object.

% Import an STL mesh, returning a PATCH-compatible face-vertex structure
fv = stlread('Qolo_w_user_01.stl');
%fv = reducepatch(fv, 0.1);


%% Render
% The model is rendered with a PATCH graphics object. We also add some dynamic
% lighting, and adjust the material properties to change the specular
% highlighting.

patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);

%% Load contact points

result = load("qolo_contact_points_case_3.mat");
X = result.result(:,4);
Y = result.result(:,5);
Z = result.result(:,6);
human_link_indices = result.result(:,2);

red = ones(length(human_link_indices), 1)*[1, 0, 0];
blue = ones(length(human_link_indices), 1)*[0, 1, 0];
green = ones(length(human_link_indices), 1)*[0, 0, 1];
yellow = ones(length(human_link_indices), 1)*[1, 1, 0];
pink = ones(length(human_link_indices), 1)*[1, 0, 1];
cyan = ones(length(human_link_indices), 1)*[0, 1, 1];
black = ones(length(human_link_indices), 1)*[0, 0, 0];
grey_white = ones(length(human_link_indices), 1)*[0.8, 0.8, 0.8];
orange = ones(length(human_link_indices), 1)*[255/255, 110/255, 10/255];
purple = ones(length(human_link_indices), 1)*[110/255, 0, 1];

C = zeros(length(human_link_indices), 3);

true_for_foot_indices = (...
    (human_link_indices == 6) |...
    (human_link_indices == 7) |...
    (human_link_indices == 16) |...
    (human_link_indices == 17) |...
    (human_link_indices == 18) |...
    (human_link_indices == 19));
true_for_shin_indices = (...
    (human_link_indices == 4) |...
    (human_link_indices == 5));
true_for_leg_indices = (...
    (human_link_indices == 2) |...
    (human_link_indices == 3));
true_for_arm_indices = (...
    (human_link_indices == 8) |...
    (human_link_indices == 9));
true_for_forearm_indices = (...
    (human_link_indices == 10) |...
    (human_link_indices == 11));
true_for_hand_indices = (...
    (human_link_indices == 12) |...
    (human_link_indices == 13));
true_for_head_indices = (human_link_indices == 15);
true_for_neck_indices = (human_link_indices == 14);
true_for_torso_indices = ((human_link_indices == -1) |...
    (human_link_indices == 0));
true_for_pelvis_indices = (human_link_indices == 1);

C(true_for_foot_indices, :) = red(true_for_foot_indices, :);
C(true_for_shin_indices, :) = orange(true_for_shin_indices, :);
C(true_for_leg_indices, :) = yellow(true_for_leg_indices, :);
C(true_for_hand_indices, :) = blue(true_for_hand_indices, :);
C(true_for_forearm_indices, :) = cyan(true_for_forearm_indices, :);
C(true_for_arm_indices, :) = grey_white(true_for_arm_indices, :);
C(true_for_pelvis_indices, :) = pink(true_for_pelvis_indices, :);
C(true_for_torso_indices, :) = green(true_for_torso_indices, :);
C(true_for_neck_indices, :) = black(true_for_neck_indices, :);
C(true_for_head_indices, :) = purple(true_for_head_indices, :);

hold on
scatter3(X, Y, Z, 20, C, 'filled')


