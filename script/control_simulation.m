clear all 
%close all

% 3 controller parameters
actuator_force_max = 0;%600; %6000; % set it to zero for no control
actuator_force_rise_time = 0.001;
actuator_delay = 0.000;
k_shield = 20000;
k_qolo = 30000;
k_computer = 30000;
k_wheel = 10000;
k_hand = 75000;

        k_shield = 4000;

        k_qolo = 8000;

        k_computer = 10000;

        k_wheel = 1000;

        k_hand = 75000;
                k_shield = 10000;
                k_qolo = 10000;
                k_computer = 10000;
                k_wheel = 1000;
                k_hand = 75000;
robot_spring_constants = [k_shield, k_qolo,k_computer, k_wheel, k_hand];

phase_1_output = load('result_phase_1.mat');
F_contact_peak_per_iteration = zeros(size(phase_1_output.result, 1), 1);
F_ref_per_iteration = zeros(size(phase_1_output.result, 1), 1);
alignment_normal_axle_per_iteration = zeros(size(phase_1_output.result, 1), 1);
F_threshold_per_iteration = zeros(size(phase_1_output.result, 1), 1);
for i = 1:size(phase_1_output.result, 1)
    [F_contact_peak, F_ref, alignment_normal_axle, F_threshold] = simulate_collision_condition_from_phase_1(phase_1_output.result(i,:), ...
        actuator_force_max, actuator_force_rise_time, actuator_delay, robot_spring_constants);
    F_contact_peak_per_iteration(i) = F_contact_peak;
    F_ref_per_iteration(i) = F_ref;
    alignment_normal_axle_per_iteration(i) = alignment_normal_axle;
    F_threshold_per_iteration(i) = F_threshold;
end
return
hot_cmap = colormap('hot');
color_coding = hot_cmap(max(min(round(15*F_contact_peak_per_iteration./F_threshold_per_iteration),size(hot_cmap,1)), 1), :);
F_normalize_per_iteration = F_contact_peak_per_iteration./F_threshold_per_iteration;
save('qolo_case_4_force_peaks_phase_2_child.mat', 'F_contact_peak_per_iteration', 'F_normalize_per_iteration', 'color_coding')

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