function [F_contact_peak, F_ref, alignment_normal_axle, F_threshold] = simulate_collision_condition_from_phase_1(phase_1_output_of_one_iteration, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay, robot_spring_constants)
% phase_1_output_of_one_iteration:
%   [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, velocity-point1-x,-y,-z,
%    contact_normal_2_to_1-x,-y,-z, robot_speed, robot_angle, human_angle, initial_gait_phase]
v_rel = [phase_1_output_of_one_iteration(10); phase_1_output_of_one_iteration(11)];
cn_global = [phase_1_output_of_one_iteration(13); phase_1_output_of_one_iteration(14)];

% get the force threshold
F_threshold = force_threshold_by_link_indices(phase_1_output_of_one_iteration(2));

if v_rel'*cn_global > 0
    F_contact_peak = 0;
    F_ref = 0;
    alignment_normal_axle = nan;
    return
end

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
spring_constant = get_spring_constant_by_link_indices(phase_1_output_of_one_iteration(2), ...
    phase_1_output_of_one_iteration(7:9), robot_spring_constants);
m_h	= reference_mass_by_link_indices(phase_1_output_of_one_iteration(2))*phase_1_output_of_one_iteration(20);

% initial state (explanation below in the ODE function)
z_init = [0,0, y_r_initial,-robot_speed, -pi/2,0, x_h_initial,v_h_initial(1), y_h_initial,v_h_initial(2)];

% integrate the ODE
t_step = 0.001;
options = odeset('Events', @(t,z) ODE_stopping_events_function(t,z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay));
[T,Z] = ode45(@(t, z) ODE_rigid_body_and_point_on_a_plane(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay), ...
    0:t_step:0.5, z_init', options);

% get force peak
F_contact_peak = 0;
Fc_mag_vec = zeros(length(T),1);
for i = 1:length(T)
    [Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
        y_contact_point_on_robot] = compute_contact( ...
            T(i), Z(i,:)', x_initial_contact_point, y_initial_contact_point, ...
            x_contact_normal, y_contact_normal, spring_constant);
    Fc_mag = sqrt(Fx_contact_on_robot^2 + Fy_contact_on_robot^2);
    if Fc_mag > F_contact_peak
        F_contact_peak = Fc_mag;
    end
    Fc_mag_vec(i) = Fc_mag;
end

% compute it according to simple analytic formula
F_ref = -v_rel'*cn_global*sqrt(1/(1/m_h + 1/m_r)*spring_constant);

% compute alignment between collision normal and wheel axle
alignment_normal_axle = abs(y_contact_normal);

% if F_contact_peak > 1200
%     x_contact_normal
%     y_contact_normal
%     v_rel'*cn_global
%     v_rel
% end

% animate (optional)
return
slow_motion_factor = 10;
clf
animate_trajectory(t_step, slow_motion_factor,Z, x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, L, D)
end

function k = get_spring_constant_by_link_indices(human_link_idx, qolo_point, robot_spring_constants)
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
    k_h = human_link_spring_constant_map(floor(human_link_idx) + 1 + 1); % +1 for base +1 for matlab
    qolo_index = classify_qolo_point(qolo_point);
    k_r = robot_spring_constants(qolo_index);
    k = 1/(1/k_h + 1/k_r);
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

function ft = force_threshold_by_link_indices(human_link_idx)
	force_threshold_map = [ ...
		110,110,110, ... # chest belly pelvis (front)
		220,220, ... # upper legs
		130,130, ... # shins
		140,140, ... # ankles/feet
		140,140, ... # upper arms
		140,140, ... # forearms
		140,140, ... # hands
		1, ... # neck (front)
		65, ... # head (front/face)
		140,140, ... # soles/feet
		140,140, ... # toes/feet
		110,110,110, ... # chest belly pelvis (back)
		150, ... # neck (back)
		130 ... # head (back/skull)
		];
	ft = force_threshold_map(floor(human_link_idx) + 1 + 1); % #+1 for base +1 for matlab
end

function index = classify_qolo_point(qolo_point)
% decision tree

if is_computer_box(qolo_point)
    index = 3; % computer box (front or back)
    return
end
if is_wheel(qolo_point)
    index = 4; % wheel
    return
end
if is_arm_or_hand(qolo_point)
    index = 5; % arm or hand
    return
end

if qolo_point(3) < 0.2606
    index = 1; % protecive shield
    return
else
    index = 2; % upper part of qolo
    return
end
end

function res = is_computer_box(qolo_point)
res_front = in_bounding_box(qolo_point, [-0.1334, 0.136],[-0.425, -0.16],[0.254, 0.552]);
res_back = in_bounding_box(qolo_point, [-0.154, 0.118],[0.1838, 0.457],[0.263, 0.5623]);
res = res_front || res_back;
end

function res = is_wheel(qolo_point)
res_right = in_bounding_box(qolo_point, [-0.3071, -0.2247],[-0.452, -0.08556],[0.2679, 0.4101]);
res_left = in_bounding_box(qolo_point, [0.2247, 0.3071],[-0.452, -0.08556],[0.2679, 0.4101]);
res = res_right || res_left;
end

function res = is_arm_or_hand(qolo_point)
res_right = in_bounding_box(qolo_point, [-0.354,-0.2715],[-0.247,0.1211],[0.7507, 1.531]);
res_left = in_bounding_box(qolo_point, [0.2715, 0.354],[-0.247,0.1211],[0.7507, 1.531]);
res = res_right || res_left;
end

function res = in_bounding_box(qolo_point, box_limits_x, box_limits_y, box_limits_z)
lims = [box_limits_x(1), box_limits_x(2); box_limits_y(1), box_limits_y(2); box_limits_z(1), box_limits_z(2)];
res = true;
for i = 1:3
    if (lims(i,1) > qolo_point(i)) || (lims(i,2) < qolo_point(i))
        res = false;
    end
end
end

function [value, isterminal, direction] = ODE_stopping_events_function(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay)
value = zeros(2,1);
direction = zeros(2,1);
isterminal = zeros(2,1);

% contact force and point where it acts on the robot
[Fx_contact_on_robot, Fy_contact_on_robot, x_contact_point_on_robot, ...
    y_contact_point_on_robot] = compute_contact( ...
        t, z, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal, spring_constant);
value(1) = Fx_contact_on_robot^2 + Fy_contact_on_robot^2;
direction(1) = -1;
isterminal(1) = 1;

R = sqrt((x_contact_point_on_robot - z(7))^2 + ...
    (y_contact_point_on_robot - z(9))^2);
is_distant = R > 0.2;
is_free = Fx_contact_on_robot^2 + Fy_contact_on_robot^2 == 0;
value(2) = double(is_free && is_distant);
direction(2) = 0;
isterminal(2) = 1;
end

%% for the coupled simulation

function rhs = ODE_rigid_body_and_point_on_a_plane(t, z, m_r, i_r, m_h, ...
    x_initial_contact_point, y_initial_contact_point,...
    x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay)
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
    x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay);
rhs(2) = Fx_r/m_r;
rhs(4) = Fy_r/m_r;
rhs(6) = Mz_r/i_r;
rhs(8) = Fx_h/m_h;
rhs(10) = Fy_h/m_h;
end

function [Fx_r, Fy_r, Mz_r, Fx_h, Fy_h] = get_external_forces_and_moments(t, z, m_r, i_r, ...
    x_initial_contact_point, y_initial_contact_point, x_contact_normal, y_contact_normal, spring_constant, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay)
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
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, m_r, i_r, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal);
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

attraction_point = [x_h; y_h] - penetration*contact_normal_tf;
contact_force_on_human = spring_constant*(attraction_point - [x_h; y_h]);
x_contact_point_on_robot = attraction_point(1);
y_contact_point_on_robot = attraction_point(2);

if penetration < 0
    Fx_contact_on_robot = 0;
    Fy_contact_on_robot = 0;
else
    Fx_contact_on_robot = -contact_force_on_human(1);
    Fy_contact_on_robot = -contact_force_on_human(2);
end
end

function [Fx_r, Fy_r, Mz_r] = get_external_forces_and_moments_robot(t,z,Fx_contact_on_robot,...
    Fy_contact_on_robot, x_contact_point_on_robot, y_contact_point_on_robot, m_r, i_r, D, L, ...
    actuator_force_max, actuator_force_rise_time, actuator_delay, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal)
x_r = z(1);
y_r = z(3);
phi_r = z(5);

% actuator forces
[Frw, Flw] = get_wheel_forces(t, z, actuator_force_max, actuator_force_rise_time, actuator_delay, ...
    x_initial_contact_point, y_initial_contact_point, x_contact_normal, y_contact_normal);
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

function [Frw, Flw] = get_wheel_forces(t, z, actuator_force_max, ...
    actuator_force_rise_time, actuator_delay, x_initial_contact_point, y_initial_contact_point, ...
        x_contact_normal, y_contact_normal)
% use equal braking on both wheels (restriction for now)
t_after_delay = max([0,t-actuator_delay]);
braking_force = min([t_after_delay/actuator_force_rise_time, 1])*actuator_force_max;

% apply full force forward or backward dependent on the contact normal
if x_contact_normal < 0 % backward
    Frw = -braking_force;
    Flw = -braking_force;
else % forward
    Frw = braking_force;
    Flw = braking_force;
end

return
% apply some rotation control
if abs(z(5) + pi/2) > pi/20
    rotation_control = max([min([ -1000*(abs(z(5) + pi/2)-pi/20), ...
        actuator_force_max]), -actuator_force_max]);
    if z(5) + pi/2 < -pi/10
        rotation_control = -rotation_control;      
    end
    if Frw > 0
        if rotation_control > 0
            Flw = Flw - 2*rotation_control;
        else
            Frw = Frw + 2*rotation_control;
        end
    else
        if rotation_control > 0
            Frw = Frw + 2*rotation_control;
        else
            Flw = Flw - 2*rotation_control;
        end
    end
end

return
% transition to damping out the robot's velocity (halting)
f_damping = max([min([ -0.5*100*[cos(z(5)) sin(z(5))]*[z(2); z(4)], ...
    actuator_force_max]), -actuator_force_max]);

%f_damping = 0; % zero force instead of damping

t_transition = 0.1;
if t_after_delay > t_transition
    weight = min([1, (t_after_delay-t_transition)/actuator_force_rise_time]);
    Frw = Frw*(1-weight) + weight*f_damping;
    Flw = Flw*(1-weight) + weight*f_damping;
end
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
