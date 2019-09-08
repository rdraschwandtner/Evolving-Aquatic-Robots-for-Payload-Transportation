function analysis()
clear all
clear screen
close all


set(0,'defaultfigurecolor',[1 1 1])
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(9,'w',func);

% set matlab output window to long format
format long


figure;
subplot(2,1,1);
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress of the best individual')

% files = rdir('*.out')
% [numoffiles, unused] = size(files);
% for i= 1:numoffiles
%     %files(i).name
%     %[ampls, freqs, phis] = readparams(files(i).name);
%     fileName = files(i).name
%     [gens, fitnesses] = readoutfile(fileName);
%     
%     avgfitnesspergen = [];
%     bestindividualfitness = max(fitnesses);
%     bestindividualidx = find(fitnesses == bestindividualfitness);
%     plot(gens, fitnesses);
%     legendstrs{i} = strcat('replicate ', fileName);   
% end
% legend(legendstrs)
legendstr = {};
replicates = [0];
i = 1;
for replicatenum = replicates
    bestinindivfile = sprintf('./%d/%d_best_individuals_logging.dat', replicatenum, replicatenum);
    [gens, individuals, fitnesses] = readoutfile(bestinindivfile);
    plot(gens, fitnesses,'color', colors(i,:));
    legendstr{i} = sprintf('replicate #%d', replicatenum);
    i = i + 1;
end
legend(legendstr);
ylim([0 50]);

hold off;
subplot(2,1,2);
hold all;
grid on;
xlabel('generation #')
ylabel('avg fitness')
title('avg fitness progress')
i = 1;
for replicatenum = replicates
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

    plot(firstgen:lastgen, avgfitpergen,'color', colors(i,:));
    i = i+1;
end
ylim([0 50]);

% print averager
%figure;
% subplot(2,1,2);
% %hold on;
% hold all; % for automatic plot coloring
% grid on;
% xlabel('generation #')
% ylabel('avg fitness')
% title('avg fitness progress')
% 
% % r.. replicates
% numofreplicates = 8
% for r= 0:numofreplicates-1
%     avgfitnesspergen = [];
%     for gennum= 0:200-1
%         repl_genstatfilename = sprintf('%d/gen%dstats.json', r, gennum);
%         [gens, ind, fitnesses] = readgenstatsfile(repl_genstatfilename);
%         avgfitnesspergen = [avgfitnesspergen; gennum, mean(fitnesses)];
%     end
%     plot(avgfitnesspergen(:,1), avgfitnesspergen(:,2));
% end
% %legend(legendstrs)
% 
% hold off;

end


function [generation,individuals,fitnesses] = readoutfile(fname)
    inputfile = fopen(fname);
    %Headerlines = 1;
    
    values = textscan(inputfile, 'Generation: %d Individual is: %d Fitness is: %f');
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{3}; 
end

function [generation,individuals,fitnesses] = readgenstatsfile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile, '%d,%d,%d,%d,%d,%f', 'headerLines', Headerlines);
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{6};
end