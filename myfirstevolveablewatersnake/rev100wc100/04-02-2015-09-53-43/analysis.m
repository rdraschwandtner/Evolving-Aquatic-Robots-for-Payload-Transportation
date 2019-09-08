function analysis()
clear all
clear screen
%close all

% set matlab output window to long format
format long

figure;
subplot(2,1,1);
hold on;
%hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress of the best individual')

files = rdir('*.out')
[numoffiles, unused] = size(files);
numofreplicates = 18
cmap = hsv(numofreplicates);
replreadorder = [];
for i= 1:numoffiles
    %files(i).name
    %[ampls, freqs, phis] = readparams(files(i).name);
    fileName = files(i).name
    [gens, fitnesses] = readoutfile(fileName);
    
    avgfitnesspergen = [];
    bestindividualfitness = max(fitnesses);
    bestindividualidx = find(fitnesses == bestindividualfitness);
    plot(gens, fitnesses, 'Color',cmap(i,:));
    legendstrs{i} = strcat('replicate ', fileName);
    scan = textscan(fileName,'%d.out');
    replreadorder = [replreadorder scan{1}];
end
legend(legendstrs)

hold off;

% print averager
%figure;
subplot(2,1,2);
%hold on;
hold on;
grid on;
xlabel('generation #')
ylabel('avg fitness')
title('avg fitness progress')

% r.. replicate
i = 1;
for r= replreadorder   
    avgfitnesspergen = [];
    for gennum= 0:1000-1
        repl_genstatfilename = sprintf('%d/gen%dstats.json', r, gennum);
        [gens, ind, fitnesses] = readgenstatsfile(repl_genstatfilename);
        avgfitnesspergen = [avgfitnesspergen; gennum, mean(fitnesses)];
    end
    plot(avgfitnesspergen(:,1), avgfitnesspergen(:,2), 'Color',cmap(i,:));
    i = i + 1;
end

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