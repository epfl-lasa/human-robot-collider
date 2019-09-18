    clear all 
    close all
    clc

    nfig = 1;
    phase_1_output = load('qolo_contact_points_case_4_with_velocities.mat');

    % 3 controller parameters
    braking_force(1)=0;     % No control
    braking_force(2)=6000;  % Full Brake 
    braking_force(3)=1000;  % Controlled braking - Admittance simulation 
    
    for kk=2:2
        actuator_force_max = braking_force(kk);
        % Actuation rise time to full force in braking => range from acceleration
        % Values to test:   1/1000, 1/500, 1/400, 1/200, 1/100
%         samples_delay_r = [1/1000, 1/500, 1/400, 1/200, 1/100];
        samples_delay_r = linspace(0.001, 0.1, 20);
        l_delay_r = length(samples_delay_r);
        
        for jj=1:l_delay_r
            actuator_force_rise_time = samples_delay_r(jj);
            % Delay in control: range from 500 Hz to 10 Hz
            % Values to test:     1/500, 1/400, 1/200, 1/100, 1/50, 1/20, 1/10
%             actuator_delay = (1/400) + (1/500);
%             samples_delay_c = [1/1000, 1/500, 1/400, 1/200, 1/100, 1/50, 1/20, 1/10];
            samples_delay_c = linspace(0.001, 0.1, 20);
            l_delay_c = length(samples_delay_c);
            for ii=1:l_delay_c
                actuator_delay = samples_delay_c(ii);
                
                k_shield = 10000;
                k_qolo = 10000;
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

                F_peak{ii}{jj} = F_contact_peak_per_iteration;
                F_peak_norm{ii}{jj} = F_contact_peak_per_iteration./F_threshold_per_iteration;
                
                % Rows - Control delay time 
                % Colmuns -- Actuators reaction time
                F_mean(ii,jj) = mean(F_contact_peak_per_iteration);
                F_std(ii,jj) = std(F_contact_peak_per_iteration);
                F_norm_mean(ii,jj) = mean(F_peak_norm{ii}{jj});
                F_norm_std(ii,jj) = std(F_peak_norm{ii}{jj});
            end
        end
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

save('Delays_Data_2_20x20.mat');

%%  Plotting Data

clc
clear all
close all
% load('Delays_Data_2_20x20.mat');
load('Delays_Data_2_20x20_both.mat');

%%
    nfig = nfig +1;
    plotHeatMap(nfig,flipud(F_norm_mean'),samples_delay_c',flipud(samples_delay_r'),'Delays_simulation_2',1)

    % load('Delays_Data_3_20x20.mat');  % Method 3 controlled braking
%     plotHeatMap(nfig,flipud(F_norm_mean),samples_delay_c',flipud(samples_delay_r'),'Delays_simulation_3',1)
    
    function plotHeatMap(nfig,F_peak,Xsamples,Ysamples,FigName,RECORD)
    
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
        Xlabel = 'Control Delay [s]';
        Ylabel = 'Actuation Delay [s]';
%         Legends = {'No detection','Emergency Braking','Controlled Braking'};
        FaceALphas = 0.2;
        FontSizes = 38;
        MarkersSizes = 14;
        LinesWidths = 2;
        FigureFile = 'epsc';
        Fonts = 'Times New Roman';
        
        figH = figure(nfig);
        set(gcf, 'name', FigName);
%         set(gcf, 'Position', [10 10 1080 780]);
        set(gca,'FontName',Fonts,...
                'FontSize', FontSizes,...
                'LineWidth',LinesWidths);
        hold on; grid on;
        
        Nsizex = 500; % Heatmap size - inerpolation
        Nsizey = 500; % Heatmap size - inerpolation
        NxLab = 8;
        NyLab = 8;
        heatMap = imresize(F_peak, [Nsizex, Nsizey]);
        imshow(heatMap, []);
        axH=figH.CurrentAxes;
%         axH = gca;
        axis on;
%         colormap(hot);      % hot gray  bone hot(256)
%         imagesc(F_peak);
        cmap = colormap(hot(256));
        CB = colorbar;
        caxis([min(min(F_peak))*1,...
                max(max(F_peak))*1.0]);
        axis([0 Nsizex 0 Nsizey]);

%         cmap2 = colormap( flipud(cmap) );
        
%         axI = figN.CurrentAxes;
        % Get the current axes labels
        Lx = axH.XTickLabel;
        Ly = axH.YTickLabel;
        % Fill new labels with the desired real heatmap axes
        Lx0=linspace(1,Nsizex,NxLab);%Nsizex);
        Ly0=linspace(1,Nsizex,NyLab);%Nsizex);
        Lx2=linspace(Xsamples(1),Xsamples(end),NxLab);%Nsizex);
        Ly2=linspace(Ysamples(1),Ysamples(end),NyLab);

        Lx2 = round(Lx2,2);
        Ly2 = round(Ly2,2);
        % Put the data to the labels
        Lx_cell={};
        for k=1:1:NxLab
            Lx1=num2str(Lx2(k));
            Lx_cell=[Lx_cell Lx1];
        end
        Lx_cell=Lx_cell';
        
        Ly_cell={};
        for k=1:1:NyLab
            Ly1=num2str(Ly2(k));
            Ly_cell=[Ly_cell Ly1];
        end
        Ly_cell=Ly_cell';
        
%         axH.XTickLabel = Lx_cell;
%         axH.YTickLabel = Ly_cell;
%         figH.CurrentAxes.XTickLabel = Lx_cell;
%         figH.CurrentAxes.YTickLabel = Ly_cell;

        set(gca,'xtick',Lx0);
        set(gca,'XtickLabel',Lx_cell,...
            'FontSize',FontSizes-4,...
            'FontName','Times New Roman'); %,'FontWeight','bold');

        set(gca,'ytick',Ly0);
        set(gca,'YtickLabel',Ly_cell,...
            'FontSize',FontSizes,...
            'FontName','Times New Roman'); %,'FontWeight','bold');
        
        set(gcf, 'name', FigName)
        set(gcf, 'Position', [0 0 1280 680]);     
        
        hXLabel = xlabel(Xlabel);
        hYLabel = ylabel(Ylabel);
        
%         set(gca,'XtickLabel',FeatureNames(ind),...
%             'FontSize',FontSize-4,'FontName','Times New Roman'); %,'FontWeight','bold');
%         set(gca,'xtick',Xsamples);
%         set(gca,'YtickLabel',FeatureNames(ind),...
%             'FontSize',FontSize-4,'FontName','Times New Roman'); %,'FontWeight','bold');
%         set(gca,'ytick',Ysamples);
        ylabel(CB, 'Peak Force to Pain Limit ratio','FontSize',FontSizes)
        set(gcf,'PaperPositionMode', 'auto');
        set([hXLabel, hYLabel]  , ...
                'FontName',  Fonts,...
                'FontSize',  FontSizes,...
                'color',     [0 0 0]);
    %     title(FigName)

        if (RECORD) 
            set(gcf,'PaperPositionMode', 'auto');   % Required for exporting graphs
            saveas(nfig,strcat(figPath,FigName),FigureFile);
        end
        hold off;
    end