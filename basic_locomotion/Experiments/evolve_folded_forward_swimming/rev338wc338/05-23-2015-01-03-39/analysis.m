function analysis()
clear all
clear screen
close all

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

legendstr = {};
for replicatenum = 0:17
    bestinindivfile = sprintf('./%d/%d_best_individuals_logging.dat', replicatenum, replicatenum);
    [gens, individuals, fitnesses] = readoutfile(bestinindivfile);
    plot(gens, fitnesses);
    legendstr{replicatenum+1} = sprintf('%d', replicatenum);
end
legend(legendstr);

hold off;
subplot(2,1,2);
hold all;
grid on;
xlabel('generation #')
ylabel('avg fitness')
title('avg fitness progress')

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

    plot(firstgen:lastgen, avgfitpergen);   
end


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