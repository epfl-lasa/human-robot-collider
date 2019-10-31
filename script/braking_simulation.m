    clear all 
    close all
    clc

    nfig = 1;
    phase_1_output = load('qolo_contact_points_case_4_with_velocities.mat');

    % 3 controller parameters
    braking_force(1)=0;     % No control
    braking_force(2)=6000;  % Full Brake 
    braking_force(3)=600;  % Controlled braking - Admittance simulation 
    
    for kk=1:3
        actuator_force_max = braking_force(kk);
        % Actuation rise time to full force in braking => range from acceleration
        % Values to test:     1/500, 1/400, 1/200, 1/100
        actuator_force_rise_time = 0.001;
        % Delay in control: range from 500 Hz to 10 Hz
        % Values to test:     1/500, 1/400, 1/200, 1/100, 1/50, 1/20, 1/10
        actuator_delay = (1/400) + (1/500);

        k_shield = 4000;
        k_qolo = 8000;
        k_computer = 10000;
        k_wheel = 1000;
        k_hand = 75000;
        robot_spring_constants = [k_shield, k_qolo,k_computer, k_wheel, k_hand];

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

        % remove near misses
        near_miss_indicator_vector = F_contact_peak_per_iteration == 0;
        F_contact_peak_per_iteration(near_miss_indicator_vector) = [];
        F_ref_per_iteration(near_miss_indicator_vector) = [];
        alignment_normal_axle_per_iteration(near_miss_indicator_vector) = [];
        F_threshold_per_iteration(near_miss_indicator_vector) = [];
        
        F_peak{kk} = F_contact_peak_per_iteration;
        F_peak_norm{kk} = F_contact_peak_per_iteration./F_threshold_per_iteration;
    end
    
% figure(20)
% hold on
% histogram(F_peak{kk}, 'FaceAlpha', 0.5, 'FaceColor', [rand,rand,rand])
% xlabel('Contact Force Peak [N]')
% ylabel('Occurrence Count')
% title('Distribution of the Contact Force Peak from Vehicle Simulation')
% 
% figure(21)
% hold on
% plot(F_ref_per_iteration, F_contact_peak_per_iteration,'o')
% forces_min = min([F_ref_per_iteration; F_contact_peak_per_iteration]);
% forces_max = max([F_ref_per_iteration; F_contact_peak_per_iteration]);
% plot([forces_min, forces_max], [forces_min, forces_max], 'r')
% xlabel('Analytic Force Reference [N]')
% ylabel('Simulation Contact Force Peak [N]')
% title('Contact Force Peak Analytic vs. Simulation')
% daspect([1 1 1])

% figure(22)
% plot(alignment_normal_axle_per_iteration, F_contact_peak_per_iteration-F_ref_per_iteration, 'o')
% xlabel('Alignment Between Contact Normal and Wheel Axle []')
% ylabel('Difference Simulated and Analytic Peak Force [N]')
% title('Reason for Difference Between Formula and Simulation')


%%  Plotting Data


    nfig = nfig +1;
    plotHistogram(nfig,F_peak_norm,1)
    
    function plotHistogram(nfig,F_peak,RECORD)
    
        colm1=[51/255 153/255 255/255];
        colm2=[153/255 153/255 0/255];
        colm3=[0/255 102/255 204/255];
        colm4=[0/255 204/255 102/255];
        colm5=[255/255 128/255 0/255];
        colm6=[204/255 0/255 0/255];
        colm7=[0/255 52/255 222/255];
        colm8=[5/255 5/255 5/255];
        load('ColorsData');     % colm# , Bcol#, Rcol#, Ocol#, Gcol# --> # 1-8
        
        if ismac
            figPath = 'Figures/';
        else
            figPath = 'Figures\';
        end
        FigName = 'Braking_Methods_Differences';
        Xlabel = 'Peak-Force-to-Pain-Limit-Ratio';
        Ylabel = 'Occurrence Count';
        Legends = {'No detection','Emergency Braking',...
                        'Controlled Braking', 'Threshold'};

        FaceALphas = 0.2;
        FontSizes = 28;
        MarkersSizes = 14;
        LinesWidths = 2;
        FigureFile = 'epsc'; %'epsc';
        Fonts = 'Times New Roman';
        figure(nfig);
        set(gcf, 'name', FigName);
        set(gcf, 'Position', [10 10 780 480]);
        set(gca,'FontName',Fonts,...
                'FontSize', FontSizes,...
                'LineWidth',LinesWidths);

      %title(title);
        hold on;
        grid on;
        hYLabel = ylabel(Ylabel);
        hXLabel = xlabel(Xlabel);

        H2 = histogram(F_peak{2},30, ...% Emergency Brake
                'FaceAlpha', FaceALphas+0.2, 'FaceColor',Gcol6);  
        H3 = histogram(F_peak{3},30, ...% Controlled Braking
                'FaceAlpha', FaceALphas+0.1, 'FaceColor',Rco6 );        
        H1 = histogram(F_peak{1},30, ...% No detection
                'FaceAlpha', FaceALphas+0.1, 'FaceColor',Bcol6 ); % ,'edgecolor','none');
        
        Pain_T = plot([1 1],[0 200],'--r','LineWidth',LinesWidths+1);
    %     box off
    %     axis tight
    %     legalpha('No control','Emergency Braking','Controlled Braking',...
    %             'location','northwest')
    %     legend boxoff
        hLegend = legend([H1, H2, H3, Pain_T], ...
                  Legends, ...
                  'FontName',Fonts,...
                  'FontSize', FontSizes,'FontWeight','bold',...
                  'orientation', 'vertical',...
                  'location', 'northeast' );

        set([hXLabel, hYLabel]  , ...
                'FontName',  Fonts,...
                'FontSize',  FontSizes,...
                'color',     [0 0 0]);
    %     title(FigName)        
        if (RECORD) 
            set(gcf,'PaperPositionMode', 'auto');   % Required for exporting graphs
            saveas(nfig,strcat(figPath,FigName),FigureFile);                        
%             set(gcf,'Units','inches');
%             screenposition = get(gcf,'Position');
%             set(gcf,...
%                 'PaperPosition',[0 0 screenposition(3:4)],...
%                 'PaperSize',[screenposition(3:4)]);
%             print -dpdf -painters -fillpage -r300 Figure   % -bestfit ir -fillpage

        end
        hold off;
    end