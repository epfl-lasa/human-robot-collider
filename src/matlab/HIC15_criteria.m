% % Computing HIC (Head Injury Criteria)
% Author: Diego F. Paez G.
% Date: 1 Dec 2020
%% HIC_15 v1.1
% 
% Inputs:
%           Acc  :      [REQ'D]    [m x 1]{double};   Column-wise vector of acceleration data in [g]
%           time    :   [REQ'D]    [m x 1]{double};   Column-wise vector of sampling times in [ms]
%           coeff     : [OPTIONAL]  scalar{double};   Coeeficient from H3 dummy to others (DEFAULT = 1)
%                   'Q1.5 = ' or 'Q3=0.71' or 'Q6'
%           ShowPlots:  [OPTIONAL]  {logical}    ;    Plot filt/orig signals (DEFAULT = FALSE)
%           
% Outputs:
%           hic_value : scalar {double} value of HIC for the given data
%           interval: : [1x2] {double}    ;    2-size with t1 start time and t2 end time
% Examples:
% minTime = -20; maxTime = 100;
% range5 = find(data_filtered.test_5.time>minTime,1):...
%                             find(data_filtered.test_5.time>maxTime,1);
% [hic, interval] = HIC15_criteria(data_raw_Q3.test_5.time(range5),...
%                                  data_raw_Q3.test_5.head.areas(range5));
%%%
function [hic_value,interval] = HIC15_criteria(time,Acc,varargin)

    %%          Parse User Inputs/Outputs                                        
    p = inputParser;
    chktime     = @(x) validateattributes(x ,{'double'},{'column'}, mfilename,'outputPath',1);
    chkAcc     = @(x) validateattributes(x ,{'double'},{'column'}, mfilename,'outputPath',2);
    % Required Inputs
    addRequired(p,'Acc'        ,chkAcc);
    addRequired(p,'time'        ,chktime);

    chkcoeff     = @(x) isnumeric(x) && isscalar(x) && (x > 0);
    
    % Optional Inputs
    addOptional(p,'coeff',      1, chkcoeff)
    addOptional(p,'ShowPlots'   ,false      ,@islogical);

    parse(p,time,Acc,varargin{:});
    
    %%  Computing the HIC-15 and interval
    time=time./1e3;
    Ts = (time(2)-time(1));

    vel = cumtrapz(time,Acc); % Velocity added from time t(1)

    len = length(Acc);
    minWindow = Ts;%0.003; %3ms
    maxWindow = 0.015; %15ms
    hic_value = 0;
    t1 = time(1);
    t2 = time(2);
    for indx_i = 1:len-minWindow/Ts-1
        t2_ini = round(min([len-1,(indx_i+minWindow/Ts)]),0);
        % Checking the input data is larger than a window of 15ms
        t2_fin = round(min([len,(t2_ini+maxWindow/Ts)]),0);
        % Checking the input data is larger than a minimum window of 3ms
        if time(t2_fin) >= (time(t2_ini) + minWindow) 
            for indx_j = t2_ini:t2_fin %indx_i+0.003/Ts)%1:(indx_i+0.015/Ts)
                    deltaT = time(indx_j)-time(indx_i);
                    % Checking the data for maxWindow 
                    if deltaT < maxWindow
                        hic_temp = deltaT*...
                            ( (vel(indx_j)-vel(indx_i)) / deltaT )^2.5;
                         if hic_temp > hic_value
                             hic_value = hic_temp;
                             t1=time(indx_i);
                             t2=time(indx_j);
                         end
                    end
            end
        end
     end
     interval = [t1, t2];
     hic_value = hic_value * p.Results.coeff;
     
     
   % =============== Plots setup ===================% 
     if p.Results.ShowPlots
            minY = min(Acc);
            maxY = max(Acc)*1.1;
            minX = interval(1)-maxWindow;
            maxX = interval(2)+maxWindow;
            AxisPlots = [minX maxX minY maxY];
            PicSize = [10 10 780 480];

            FaceALphas = 0.18;
            FontSizes = 24;
            MarkersSizes = 14;
            LinesWidths = 2.8;
        %     figureFormat = 'epsc'; %'png'
            Fonts = 'Times New Roman';
            load('ColorsData');     % colm# , Bcol#, Rcol#, Ocol#, Gcol#

            switch (3)
                case 1
                    nPalet = ["EE6677", "228833", "4477AA", "CCBB44", "66CCEE", "AA3377", "BBBBBB"]; % Tol_bright 
                case 2
                    nPalet = ["88CCEE", "44AA99", "117733", "332288", "DDCC77", "999933","CC6677", "882255", "AA4499", "DDDDDD"]; % Tol_muted 
                case 3
                    nPalet = ["BBCC33", "AAAA00", "77AADD", "EE8866", "EEDD88", "FFAABB", "99DDFF", "44BB99", "DDDDDD"];% Tol_light 
                case 4
                    nPalet = ["E69F00", "56B4E9", "009E73", "F0E442", "0072B2", "D55E00", "CC79A7", "000000" ];% Okabe_Ito 
                otherwise
            % % #From Color Universal Design (CUD): https://jfly.uni-koeln.de/color/
                    nPalet = ["E69F00", "56B4E9", "009E73", "F0E442", "0072B2", "D55E00", "CC79A7", "000000" ];% Okabe_Ito 
            end
            for iColor = 1:length(nPalet)
                colorPalet(iColor,:) = hex2rgb(nPalet(iColor),255)./255;
            end
            ColorsAIS = [50 130 0;% 0-5%
                     220 200 0;% 5-20%
                     255 150 0;% 20-50%
                     190 0 0;% 50-100%
                        ]./255;
            figure;
            hold on; grid on;
            pAcc = plot(time,Acc,...
                    '--','lineWidth',LinesWidths-0.5,'markerSize',MarkersSizes,...
                    'Color',colorPalet(3,:)...
                        );
            pHIC1 = plot([interval(1) interval(1)],[minY maxY],...
                '-','Color',ColorsAIS(1,:),...
                'LineWidth',LinesWidths ...
                );
            pHIC2 = plot([interval(2) interval(2)],[minY maxY],...
                '-','Color',ColorsAIS(2,:),...
                'LineWidth',LinesWidths ...
                );
                    
            plegends = {'Acceleration', 'lower bound', 'upper bound'};
            hLegend = legend([pAcc, pHIC1, pHIC2],...
                      plegends, ...
                      'FontName',Fonts,...
                      'FontSize', FontSizes,'FontWeight','bold',...
                      'orientation', 'vertical',...
                      'location', 'NorthEast' );
            set(gcf, 'Position', PicSize);
            set(gcf,'PaperPositionMode', 'auto');
            hold on;
            hXLabel=xlabel('time [ms]');
            hYLabel=ylabel('Acceleration [g]');
            set(gca, ...
                  'Box'         , 'off'     , ...
                  'TickDir'     , 'out'     , ... % 
                  'TickLength'  , [.02 .02] , ...
                  'XMinorTick'  , 'on'      , ...
                  'YMinorTick'  , 'on'      , ...
                  'YGrid'       , 'on'      , ...
                  'XColor'      , Gcol5, ...
                  'YColor'      , Gcol5, ...%           'XTick'       , 0:round(m/10):(m+1), ... %           'YTick'       , Ymin:round((Ymax-Ymin)/10):Ymax, ...
                  'LineWidth'   , 1.5         );
            
            set([hXLabel, hYLabel]  , ...
                    'FontName',  Fonts,...
                    'FontSize',  FontSizes,...
                    'color',     [0 0 0]);
            axis(AxisPlots);
    
     end
% %     %%
% %     % Get all the indices between the provided times
% % %     sampleIndices = find((time>=t1) & (time<=t2));
% %     sampleIndices = find((time>=t1) & (time<=t2));
% % 
% %     % Get the net time:
% %     tNet = t(sampleIndices);
% % 
% %     % Integrating the time *between* samples means you use nSamples-1 for the signal:
% %     aNet = a(sampleIndices(1:end-1));
% % 
% %     % Get the time intervals:
% %     dT = diff(t(sampleIndices));
% % 
% %     % Integrate
% %     integral = cumsum(aNet.*dT);
% % 
% %     % Do the formula:
% %     % Redefine t1 and t2 based on the time samples actually used:
% %     t2 = tNet(end);
% %     t1 = tNet(1);
% % 
% %     HIC = max( (t2-t1) * ((1/(t2-t1)) * integral).^(2.5) )



end

    