clear all 
%close all

mean_normalized_force = zeros(1000,1);
mean_force = zeros(1000,1);
k_bumper_vec = 1.005.^(0:999)*1000+100;
count = 0;

for k_bumper = k_bumper_vec
        % 3 controller parameters
        actuator_force_max = 1000; % set it to zero for no control
        actuator_force_rise_time = 0.001;
        actuator_delay = 0.000;
        k_shield = 20000;
        k_qolo = 30000;
        k_computer = 30000;
        k_wheel = 10000;
        k_hand = 75000;

        k_shield = k_bumper;

        k_qolo = 0;

        k_computer = 0;

        k_wheel = 0;

        k_hand = 0;
        robot_spring_constants = [k_shield, k_qolo,k_computer, k_wheel, k_hand];

        phase_1_output = load('qolo_contact_points_case_4_with_velocities.mat');
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

        %save('qolo_case_4_force_peaks_phase_2.mat', 'F_contact_peak_per_iteration')

        % remove near misses
        near_miss_indicator_vector = F_contact_peak_per_iteration == 0;
        F_contact_peak_per_iteration(near_miss_indicator_vector) = [];
        F_ref_per_iteration(near_miss_indicator_vector) = [];
        alignment_normal_axle_per_iteration(near_miss_indicator_vector) = [];
        F_threshold_per_iteration(near_miss_indicator_vector) = [];

        count = count + 1;
        mean_normalized_force(count) = mean(F_contact_peak_per_iteration./F_threshold_per_iteration);
        mean_force(count) = mean(F_contact_peak_per_iteration);
end

save('Stiffness_Simulation_3.mat');

figure
plot(k_bumper_vec, mean_force)
figure
plot(k_bumper_vec, mean_normalized_force)
return

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