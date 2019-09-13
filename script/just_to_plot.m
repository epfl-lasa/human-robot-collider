 
nfig = 1;
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

        FigName = 'a';

        Xlabel = 'Stiffness [N/m]';

        Ylabel = 'Mean Force Peak [N]';

        Legends = {};
RECORD = true;

        FaceALphas = 0.2;

        FontSizes = 22;

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

        hYLabel=ylabel(Ylabel);

        hXLabel=xlabel(Xlabel);

        plot(k_bumper_vec, mean_force)
        

    %     box off

    %     axis tight

    %     legalpha('No control','Emergency Braking','Controlled Braking',...

    %             'location','northwest')

    %     legend boxoff


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