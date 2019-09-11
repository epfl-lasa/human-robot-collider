clear all 
close all

phase_1_output = load('qolo_contact_points_case_4_with_velocities.mat');
F_contact_peak_per_iteration = zeros(size(phase_1_output.result, 1), 1);
for i = 1:size(phase_1_output.result, 1)
    F_contact_peak = simulate_collision_condition_from_phase_1(phase_1_output.result(i,:));
    F_contact_peak_per_iteration(i) = F_contact_peak;
end

F_contact_peak_per_iteration(F_contact_peak_per_iteration == 0) = []; % remove near misses

hist(F_contact_peak_per_iteration)
xlabel('Contact Force Peak [N]')
ylabel('Occurrence Count')
title('Distribution of the Contact Force Peak from Vehicle Simulation')

return

disp("Script for plotting the controlled dynamic response of Qolo to a collision.")

% initial state (explanation below in the ODE function)
z_init = [0,0, 0,0, pi/4,0, 0.3,-0.3, 0.3,-0.3];

% constants (explanation below in the ODE function)
m_r	= 100;
i_r	= m_r/2*0.3^2; % just an example
m_h	= 10; % just from one link
x_initial_contact_point = 0.3*sqrt(2);
y_initial_contact_point = 0;
x_contact_normal = -1;
y_contact_normal = 0;
spring_constant = 40000;
L = 0.4;
D = 0.2;

tic
% integrate the ODE
t_step = 0.001;
[T,Z] = ode45(@(t, z) ODE_rigid_body_and_point_on_a_plane(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L), 0:t_step:1, z_init');
toc

% animate the result
slow_motion_factor = 10;
animate_trajectory(t_step, slow_motion_factor,Z, x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, L, D)

% plot the resulting contact force magnitude over time
figure
Fc_mag = zeros(length(T),1);
for i = 1:length(T)
    [Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
        y_contact_point_on_robot] = compute_contact( ...
            T(i), Z(i,:)', x_initial_contact_point, y_initial_contact_point, ...
            x_contact_normal, y_contact_normal, spring_constant);
    Fc_mag(i) = sqrt(Fx_contact_on_robot^2 + Fy_contact_on_robot^2);
end
plot(T, Fc_mag)
xlabel('Time [s]')
ylabel('Force [N]')
title('Contact Force Magnitude')

%% for calling the simulation from specific initial conditions corresponding to the outputs of phase 1

function F_contact_peak = simulate_collision_condition_from_phase_1(phase_1_output_of_one_iteration)
% phase_1_output_of_one_iteration:
%   [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, velocity-point1-x,-y,-z,
%    contact_normal_2_to_1-x,-y,-z, robot_speed, robot_angle, human_angle, initial_gait_phase]

robot_speed = phase_1_output_of_one_iteration(16);
x_h_initial = phase_1_output_of_one_iteration(4);
y_h_initial = phase_1_output_of_one_iteration(5);
v_h_initial = [0; -robot_speed] + [phase_1_output_of_one_iteration(10); ...
    phase_1_output_of_one_iteration(11)];

% constants (explanation below in the ODE function)
m_r	= 100; % to be set according to Qolo data
i_r	= m_r/2*0.3^2; % to be set according to Qolo data
L = 0.4; % to be set according to Qolo data
D = 0.2; % to be set according to Qolo data

qolo_model_axle_default_y = 0.254475;
y_r_initial = D-qolo_model_axle_default_y;

% parameters (explanation below in the ODE function)
x_initial_contact_point = -y_h_initial + y_r_initial; % rotated by pi/2 and shifted
y_initial_contact_point = x_h_initial; % rotated by pi/2
x_contact_normal = phase_1_output_of_one_iteration(14); % rotated by pi/2
y_contact_normal = -phase_1_output_of_one_iteration(13); % rotated by pi/2
spring_constant = get_spring_constant_by_link_indices(phase_1_output_of_one_iteration(2), 0);
m_h	= reference_mass_by_link_indices(phase_1_output_of_one_iteration(2));

% initial state (explanation below in the ODE function)
z_init = [0,0, y_r_initial,-robot_speed, -pi/2,0, x_h_initial,v_h_initial(1), y_h_initial,v_h_initial(2)];

% integrate the ODE
t_step = 0.001;
[T,Z] = ode45(@(t, z) ODE_rigid_body_and_point_on_a_plane(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L), 0:t_step:0.5, z_init');

% get force peak
F_contact_peak = 0;
for i = 1:length(T)
    [Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
        y_contact_point_on_robot] = compute_contact( ...
            T(i), Z(i,:)', x_initial_contact_point, y_initial_contact_point, ...
            x_contact_normal, y_contact_normal, spring_constant);
    Fc_mag = sqrt(Fx_contact_on_robot^2 + Fy_contact_on_robot^2);
    if Fc_mag > F_contact_peak
        F_contact_peak = Fc_mag;
    end
end

% animate (optional)
return
slow_motion_factor = 10;
clf
animate_trajectory(t_step, slow_motion_factor,Z, x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, L, D)
end

function k = get_spring_constant_by_link_indices(human_link_idx, qolo_link_idx)
	human_link_spring_constant_map = [...
		35000.0,35000.0,35000.0, ... # chest belly pelvis (front)
		50000.0,50000.0, ...# upper legs
		60000.0,60000.0, ...# shins
		75000.0,75000.0, ...# ankles/feet
		75000.0,75000.0, ...# upper arms
		75000.0,75000.0, ...# forearms
		75000.0,75000.0, ...# hands
		50000.0, ...# neck (front)
		75000.0, ...# head (front/face)
		75000.0,75000.0, ...# soles/feet
		75000.0,75000.0, ...# toes/feet
		35000.0,35000.0,35000.0, ...# chest belly pelvis (back)
		50000.0, ...# neck (back)
		150000.0 ...# head (back/skull)
		];
    k = human_link_spring_constant_map(floor(human_link_idx) + 1 + 1); % +1 for base +1 for matlab
end

function m = reference_mass_by_link_indices(human_link_idx)
	human_link_mass_map = [ ...
		40.0,40.0,40.0, ... # chest belly pelvis (front)
		15.0,15.0, ... # upper legs
		5.0,5.0, ... # shins
		1.0,1.0, ... # ankles/feet
		3.0,3.0, ... # upper arms
		2.0,2.0, ... # forearms
		0.6,0.6, ... # hands
		1.2, ... # neck (front)
		4.4, ... # head (front/face)
		1.0,1.0, ... # soles/feet
		1.0,1.0, ... # toes/feet
		40.0,40.0,40.0, ... # chest belly pelvis (back)
		1.2, ... # neck (back)
		4.4 ... # head (back/skull)
		];
	m = human_link_mass_map(floor(human_link_idx) + 1 + 1); % #+1 for base +1 for matlab
end

%% for the coupled simulation

function rhs = ODE_rigid_body_and_point_on_a_plane(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L)
%     t     time
% positions:
    % z(1)	robot position in x (com position)
    % z(3)	robot position in y (com position)
    % z(5)	robot position in phi (angular position)
    % z(7)	human link position in x
    % z(9)	human link position in y
% velocities:
    % z(2)	robot velocity in x (com velocity)
    % z(4)	robot velocity in y (com velocity)
    % z(6)	robot velocity in phi (angular velocity)
    % z(8)	human link velocity in x
    % z(10)	human link velocity in y
% constants:
    % m_r	robot mass
    % i_r	robot moment of inertia
    % m_h	human mass
    % x_initial_contact_point, y_initial_contact_point
    %       the inital contact point in local robot coordinates
    % x_contact_normal, y_contact_normal
    %       the contact normal in local robot coordinates
    % spring_constant
    %       the spring constant used by the contact model
    % L     distance between wheels
    % D     distance between axle and COM

rhs = zeros(10,1);
% position derivatives = velocities
rhs(1) = z(2); 
rhs(3) = z(4);
rhs(5) = z(6);
rhs(7) = z(8);
rhs(9) = z(10);
% velocity derivatives = forces or moments divided by inertial terms
[Fx_r, Fy_r, Mz_r, Fx_h, Fy_h] = get_external_forces_and_moments(t, z, m_r, i_r, ... % pass inertial terms to solve for the constrainig force
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L);
rhs(2) = Fx_r/m_r;
rhs(4) = Fy_r/m_r;
rhs(6) = Mz_r/i_r;
rhs(8) = Fx_h/m_h;
rhs(10) = Fy_h/m_h;
end

function [Fx_r, Fy_r, Mz_r, Fx_h, Fy_h] = get_external_forces_and_moments(t, z, m_r, i_r, ...
    x_initial_contact_point, y_initial_contact_point, x_contact_normal, y_contact_normal, spring_constant, D, L)
% contact force and point where it acts on the robot
[Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
    y_contact_point_on_robot] = compute_contact( ...
        t, z, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal, spring_constant);

% forces acting on the human
Fx_h = -Fx_contact_on_robot;
Fy_h = -Fy_contact_on_robot;
 
% compute resultants of forces acting on the robot
[Fx_r, Fy_r, Mz_r] = get_external_forces_and_moments_robot(t,z,Fx_contact_on_robot,...
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, m_r, i_r, D, L);
end

function [Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
    y_contact_point_on_robot] = compute_contact( ...
        t, z, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal, spring_constant)
x_r = z(1);
y_r = z(3);
phi_r = z(5);
x_h = z(7);
y_h = z(9);
R = [cos(phi_r), -sin(phi_r); sin(phi_r), cos(phi_r)];
contact_normal_tf = R*[x_contact_normal; y_contact_normal];
initial_contact_point_tf = R*[x_initial_contact_point; y_initial_contact_point] + [x_r; y_r];
contact_normal_tf = contact_normal_tf/sqrt(contact_normal_tf'*contact_normal_tf); % re-normalize
penetration = contact_normal_tf'*([x_h; y_h] - initial_contact_point_tf);
if penetration < 0
    Fx_contact_on_robot = 0;
    Fy_contact_on_robot = 0;
    x_contact_point_on_robot = 0;
    y_contact_point_on_robot = 0;
    return
end
attraction_point = [x_h; y_h] - penetration*contact_normal_tf;
contact_force_on_human = spring_constant*(attraction_point - [x_h; y_h]);
Fx_contact_on_robot = -contact_force_on_human(1);
Fy_contact_on_robot = -contact_force_on_human(2);
x_contact_point_on_robot = attraction_point(1);
y_contact_point_on_robot = attraction_point(2);
end

function [Fx_r, Fy_r, Mz_r] = get_external_forces_and_moments_robot(t,z,Fx_contact_on_robot,...
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, m_r, i_r, D, L)
x_r = z(1);
y_r = z(3);
phi_r = z(5);

% actuator forces
[Frw, Flw] = get_wheel_forces(t, z);
% constraining force due to differential drive kinematics
Fconstraint = get_laterally_constraining_force(t,z, Frw, Flw, Fx_contact_on_robot,...
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, L, D, m_r, i_r);

% compute resultants (collect contributions)
FxFyMz_r = zeros(3,1);
FxFyMz_r = FxFyMz_r + [cos(phi_r); sin(phi_r); L/2]*Frw;
FxFyMz_r = FxFyMz_r + [cos(phi_r); sin(phi_r); -L/2]*Flw;
FxFyMz_r = FxFyMz_r + [-sin(phi_r); cos(phi_r); D]*Fconstraint;
FxFyMz_r = FxFyMz_r + [1; 0; -(y_contact_point_on_robot-y_r)]*Fx_contact_on_robot;
FxFyMz_r = FxFyMz_r + [0; 1; x_contact_point_on_robot-x_r]*Fy_contact_on_robot;
Fx_r = FxFyMz_r(1);
Fy_r = FxFyMz_r(2);
Mz_r = FxFyMz_r(3);
end

function Fconstraint = get_laterally_constraining_force(t,z, Frw, Flw, Fx_contact_on_robot,...
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, L, D, m_r, i_r)
x_r = z(1);
y_r = z(3);
phi_r = z(5);
vx_r = z(2);
vy_r = z(4);
omega_r = z(6);
Fconstraint = 1/(1/m_r + D^2/i_r)*(L*D/2/i_r*(-Frw+Flw) + ...
    (sin(phi_r)/m_r + (y_contact_point_on_robot-y_r)*D/i_r)*Fx_contact_on_robot + ...
    -(cos(phi_r)/m_r + (x_contact_point_on_robot-x_r)*D/i_r)*Fy_contact_on_robot + ...
    vx_r*omega_r*cos(phi_r) + vy_r*omega_r*sin(phi_r));
end

function [Frw, Flw] = get_wheel_forces(t, z)
Frw = 0;
Flw = 0;
end

%% for the visualization
function animate_trajectory(t_step, slow_motion_factor, Z, x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, L, D)
s = struct;
s.Z = Z;
s.x_initial_contact_point = x_initial_contact_point;
s.y_initial_contact_point = y_initial_contact_point;
s.x_contact_normal = x_contact_normal;
s.y_contact_normal = y_contact_normal;
s.L = L;
s.D = D;
s.slow_motion_factor = slow_motion_factor;
animation_timer = timer('UserData', s, 'ExecutionMode','fixedRate', 'period', t_step*slow_motion_factor, ...
         'TasksToExecute', size(Z,1), ...
         'TimerFcn', @animation_timer_fcn, 'StartFcn', @animation_timer_start_fcn);
start(animation_timer)
while ~strcmp(animation_timer.running, 'off')
    pause(0.1)
end
delete(animation_timer)
end

function animation_timer_start_fcn(timer_object, event)
hold on
x_r = timer_object.UserData.Z(1,1);
y_r = timer_object.UserData.Z(1,3);
timer_object.UserData.robot_line_h = plot(x_r,y_r, 'ko');

x_h = timer_object.UserData.Z(1,7);
y_h = timer_object.UserData.Z(1,9);
timer_object.UserData.human_line_h = plot(x_h,y_h, 'ro');

[X_axle, Y_axle, X_normal, Y_normal, x_init_c, y_init_c] = get_robot_fixed_shapes( ...
    timer_object.UserData.Z(1,:), timer_object.UserData.x_initial_contact_point, ...
    timer_object.UserData.y_initial_contact_point, timer_object.UserData.x_contact_normal, ...
    timer_object.UserData.y_contact_normal, timer_object.UserData.L, timer_object.UserData.D);
timer_object.UserData.axle_line_h = plot(X_axle,Y_axle, 'k');
timer_object.UserData.init_c_line_h = plot(x_init_c,y_init_c, 'go');
timer_object.UserData.normal_line_h = plot(X_normal,Y_normal, 'g');

min_x = min([timer_object.UserData.Z(:,1); timer_object.UserData.Z(:,7)]);
max_x = max([timer_object.UserData.Z(:,1); timer_object.UserData.Z(:,7)]);
min_y = min([timer_object.UserData.Z(:,3); timer_object.UserData.Z(:,9)]);
max_y = max([timer_object.UserData.Z(:,3); timer_object.UserData.Z(:,9)]);
max_delta = max([max_x-min_x, max_y-min_y]);
set(gca, 'XLim', [min_x - 1.1, min_x + max_delta + 1.1])
set(gca, 'YLim', [min_y - 1.1, min_y + max_delta + 1.1])

legend([timer_object.UserData.robot_line_h, ...
    timer_object.UserData.human_line_h, ...
    timer_object.UserData.axle_line_h, ...
    timer_object.UserData.normal_line_h, ...
    timer_object.UserData.init_c_line_h], ...
    {'Robot COM', 'Human', 'Wheel Axle', 'Contact Normal', 'Initial Contact Point'})

title("Collision Animation ("+string(timer_object.UserData.slow_motion_factor)+"x slow motion)")
end

function animation_timer_fcn(timer_object, event)
i = timer_object.TasksExecuted;
x_r = timer_object.UserData.Z(i,1);
y_r = timer_object.UserData.Z(i,3);
x_h = timer_object.UserData.Z(i,7);
y_h = timer_object.UserData.Z(i,9);
[X_axle, Y_axle, X_normal, Y_normal, x_init_c, y_init_c] = get_robot_fixed_shapes( ...
    timer_object.UserData.Z(i,:), timer_object.UserData.x_initial_contact_point, ...
    timer_object.UserData.y_initial_contact_point, timer_object.UserData.x_contact_normal, ...
    timer_object.UserData.y_contact_normal, timer_object.UserData.L, timer_object.UserData.D);
if isvalid(timer_object.UserData.robot_line_h)
    set(timer_object.UserData.robot_line_h, 'XData', x_r, 'YData', y_r);
    set(timer_object.UserData.human_line_h, 'XData', x_h, 'YData', y_h);
    set(timer_object.UserData.axle_line_h, 'XData', X_axle, 'YData', Y_axle);
    set(timer_object.UserData.init_c_line_h, 'XData', x_init_c, 'YData', y_init_c);
    set(timer_object.UserData.normal_line_h, 'XData', X_normal, 'YData', Y_normal);
    daspect([1 1 1])
else
    stop(timer_object)
    return
end
end

function [X_axle, Y_axle, X_normal, Y_normal, x_init_c, y_init_c] = get_robot_fixed_shapes( ...
    z, x_initial_contact_point, y_initial_contact_point, x_contact_normal, y_contact_normal, L, D)
x_r = z(1);
y_r = z(3);
phi_r = z(5);
R = [cos(phi_r), -sin(phi_r); sin(phi_r), cos(phi_r)];
init_c = R*[x_initial_contact_point; y_initial_contact_point] + [x_r; y_r];
x_init_c = init_c(1);
y_init_c = init_c(2);
axle_XY_local = [D, D; -L/2, L/2];
axle_XY_global = R*axle_XY_local + [x_r; y_r]*ones(1,2);
X_axle = axle_XY_global(1,:);
Y_axle = axle_XY_global(2,:);
contact_normal_global = R*[x_contact_normal; y_contact_normal];
X_normal = [x_init_c, x_init_c + contact_normal_global(1)];
Y_normal = [y_init_c, y_init_c + contact_normal_global(2)];
end
