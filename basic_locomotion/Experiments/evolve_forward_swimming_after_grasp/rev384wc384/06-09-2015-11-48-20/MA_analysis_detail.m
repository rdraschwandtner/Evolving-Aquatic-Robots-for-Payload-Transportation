
function analysis_detail()
clear all
clear screen
%close all

% set matlab output window to long format
format long

set(0,'defaultfigurecolor',[1 1 1])
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(18,'w',func);
ytick = [0:2:40];
xtick = [0:5:999,999];


figure('units','normalized','outerposition',[0 0 1 1]); %fullscreen figure;
subplot(2,1,1);
%hold on;

hold all; % for automatic plot coloring
%grid on;
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',10);
xlabel('Generation','Fontsize',15);
ylabel('Fitness','Fontsize',15);
title('Max fitness progress','Fontsize',17)

legendstr = {};
for replicatenum = 0:17
    bestinindivfile = sprintf('./%d/%d_best_individuals_logging.dat', replicatenum, replicatenum);
    [gens, individuals, fitnesses] = readoutfile(bestinindivfile);
    plot(gens, fitnesses, 'color', colors(replicatenum+1,:));
    legendstr{replicatenum+1} = sprintf('%d', replicatenum);
end
legend(legendstr,'location','southoutside','Orientation','horizontal');
breakxaxis([99 990],0.01);
hold off;
subplot(2,1,2);
hold all;
%grid on;
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',10);
xlabel('Generation','Fontsize',15);
ylabel('Fitness','Fontsize',15);
title('Mean fitness progress','Fontsize',17)

for replicatenum = 0:17
    fitnessfile = sprintf('./%d/%d_fitnesses.dat', replicatenum, replicatenum);
    [gens, ind, fitnesses] = readgenstatsfile(fitnessfile);

    lastgen = max(gens);
    firstgen = min(gens);

    avgfitpergen = [];
    for gennum= firstgen:lastgen
        gens_idc = find(gens==gennum);
        %number of gens_idc should meet the population size
        avgfitpergen(gennum+1) = mean(fitnesses(gens_idc));
    end

    plot(firstgen:lastgen, avgfitpergen, 'color', colors(replicatenum+1,:));   
end
breakxaxis([99 990],0.01);
hold off;

end


function [generation,individuals,fitnesses] = readoutfile(fname)
    inputfile = fopen(fname);
    %Headerlines = 1;
    
    values = textscan(inputfile, 'Generation: %d Individual is: %d Fitness is: (%f,)');
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{3}; 
end

function [generation,individuals,fitnesses] = readgenstatsfile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile, '%d;%d;(%d,)', 'headerLines', Headerlines);
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{3};
end