
function analysis_detail()
clear all
clear screen
%close all

% set matlab output window to long format
format long

set(0,'defaultfigurecolor',[1 1 1])
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(18,'w',func);
ytick = [0:2:50];
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
    bestinindivfile = sprintf('%d.out', replicatenum);
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
    lastgen = 999;
    firstgen = 0;

    avgfitpergen = [];
    for gennum= firstgen:lastgen
            fitnessfile = sprintf('%d/gen%dstats.json', replicatenum, gennum);
            [gens, ind, fitnesses] = readgenstatsfile(fitnessfile);
            %number of gens_idc should meet the population size
            avgfitpergen(gennum+1) = mean(fitnesses);  
    end

    plot(firstgen:lastgen, avgfitpergen, 'color', colors(replicatenum+1,:));   
end
breakxaxis([99 990],0.01);
hold off;

end


function [gens,individuals,fitnesses] = readoutfile(fname)
    inputfile = fopen(fname);
    % delimiter = '  ';  % for tabs: delimiter = sprintf('\t');
    CommentStyle = '#';
    Headerlines = 1;


    % Get the values from the file
    %values = textscan(inputfile, '%s%d%s%d%s%d', 'delimiter', delimiter, 'CommentStyle', CommentStyle);
    values = textscan(inputfile, '%s  %d  %s  %f  %s  %d', 'CommentStyle', CommentStyle, 'headerLines', Headerlines);

    fclose(inputfile);
    gens = values{2};
    fitnesses = values{4};  
    individuals = values{6};
end

function [generation,individuals,fitnesses] = readgenstatsfile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile, '%d;%d;%f', 'headerLines', Headerlines);
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{3};
end