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

files = rdir('*.out')
[numoffiles, unused] = size(files);
for i= 1:numoffiles
    %files(i).name
    %[ampls, freqs, phis] = readparams(files(i).name);
    fileName = files(i).name
    [gens, fitnesses] = readoutfile(fileName);
    
    avgfitnesspergen = [];
    bestindividualfitness = max(fitnesses);
    bestindividualidx = find(fitnesses == bestindividualfitness);
    plot(gens, fitnesses);
    legendstrs{i} = strcat('replicate ', fileName);   
end
legend(legendstrs)

hold off;

% print averager
%figure;
subplot(2,1,2);
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('avg fitness')
title('avg fitness progress')

% r.. replicates
numofreplicates = 8
for r= 0:numofreplicates-1
    avgfitnesspergen = [];
    for gennum= 0:200-1
        repl_genstatfilename = sprintf('%d/gen%dstats.json', r, gennum);
        [gens, ind, fitnesses] = readgenstatsfile(repl_genstatfilename);
        avgfitnesspergen = [avgfitnesspergen; gennum, mean(fitnesses)];
    end
    plot(avgfitnesspergen(:,1), avgfitnesspergen(:,2));
end
%legend(legendstrs)

hold off;

end


function [gens,fitnesses] = readoutfile(fname)
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