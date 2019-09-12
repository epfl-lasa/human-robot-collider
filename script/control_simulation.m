clear all 
%close all

% 3 controller parameters
actuator_force_max = 000; % set it to zero for no control
actuator_force_rise_time = 0.001;
actuator_delay = 0.000;

phase_1_output = load('qolo_contact_points_case_4_with_velocities.mat');
F_contact_peak_per_iteration = zeros(size(phase_1_output.result, 1), 1);
F_ref_per_iteration = zeros(size(phase_1_output.result, 1), 1);
alignment_normal_axle_per_iteration = zeros(size(phase_1_output.result, 1), 1);
F_threshold_per_iteration = zeros(size(phase_1_output.result, 1), 1);
for i = 1:size(phase_1_output.result, 1)
    [F_contact_peak, F_ref, alignment_normal_axle, F_threshold] = simulate_collision_condition_from_phase_1(phase_1_output.result(i,:), ...
        actuator_force_max, actuator_force_rise_time, actuator_delay);
    F_contact_peak_per_iteration(i) = F_contact_peak;
    F_ref_per_iteration(i) = F_ref;
    alignment_normal_axle_per_iteration(i) = alignment_normal_axle;
    F_threshold_per_iteration(i) = F_threshold;
end

% remove near misses
near_miss_indicator_vector = F_contact_peak_per_iteration == 0;
F_contact_peak_per_iteration(near_miss_indicator_vector) = [];
F_ref_per_iteration(near_miss_indicator_vector) = [];
alignment_normal_axle_per_iteration(near_miss_indicator_vector) = [];
F_threshold_per_iteration(near_miss_indicator_vector) = [];

figure(20)
hold on
histogram(F_contact_peak_per_iteration, 'FaceAlpha', 0.5, 'FaceColor', [rand,rand,rand])
xlabel('Contact Force Peak [N]')
ylabel('Occurrence Count')
title('Distribution of the Contact Force Peak from Vehicle Simulation')
figure(21)
hold on
plot(F_ref_per_iteration, F_contact_peak_per_iteration,'o')
forces_min = min([F_ref_per_iteration; F_contact_peak_per_iteration]);
forces_max = max([F_ref_per_iteration; F_contact_peak_per_iteration]);
plot([forces_min, forces_max], [forces_min, forces_max], 'r')
xlabel('Analytic Force Reference [N]')
ylabel('Simulation Contact Force Peak [N]')
title('Contact Force Peak Analytic vs. Simulation')
daspect([1 1 1])

% figure(22)
% plot(alignment_normal_axle_per_iteration, F_contact_peak_per_iteration-F_ref_per_iteration, 'o')
% xlabel('Alignment Between Contact Normal and Wheel Axle []')
% ylabel('Difference Simulated and Analytic Peak Force [N]')
% title('Reason for Difference Between Formula and Simulation')

figure(23)
hold on
histogram(F_contact_peak_per_iteration./F_threshold_per_iteration, 'FaceAlpha', 0.5, 'FaceColor', [rand,rand,rand])
xlabel('Peak-Force-to-Pain-Limit-Ratio')
ylabel('Occurrence Count')
title('Distribution of the Peak-Force-to-Pain-Limit-Ratio from Vehicle Simulation')