function analysis()
clear all
clear screen
close all

% set matlab output window to long format
format long

% find the best solution

fileName  = 'robonode1/0.out';
% inputfile = fopen(fileName);
% % delimiter = '  ';  % for tabs: delimiter = sprintf('\t');
% CommentStyle = '#';
% Headerlines = 1;
% 
% 
% % Get the values from the file
% %values = textscan(inputfile, '%s%d%s%d%s%d', 'delimiter', delimiter, 'CommentStyle', CommentStyle);
% values = textscan(inputfile, '%s  %d  %s  %f  %s  %d', 'CommentStyle', CommentStyle, 'headerLines', Headerlines);
% 
% fclose(inputfile);

figure;
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress on robonode 1')

for i= 0:5
    fileName = sprintf('robonode1/%d.out', i)
    [gens, fitnesses] = readoutfile(fileName);

    bestindividualfitness = max(fitnesses);
    bestindividualidx = find(fitnesses == bestindividualfitness);
    plot(gens, fitnesses);
    legendstrs{i+1} = fileName;
end
legend(legendstrs)

hold off;

figure;
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress on robonode 2')

for i= 0:5
    fileName = sprintf('robonode2/%d.out', i)
    [gens, fitnesses] = readoutfile(fileName);

    bestindividualfitness = max(fitnesses);
    bestindividualidx = find(fitnesses == bestindividualfitness);
    plot(gens, fitnesses);
    legendstrs{i+1} = fileName;
end
legend(legendstrs)

hold off;

figure;
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress on robonode 3')

for i= 0:5
    fileName = sprintf('robonode3/%d.out', i)
    [gens, fitnesses] = readoutfile(fileName);

    bestindividualfitness = max(fitnesses);
    bestindividualidx = find(fitnesses == bestindividualfitness);
    plot(gens, fitnesses);
    legendstrs{i+1} = fileName;
end
legend(legendstrs)

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