function scatteranalysis()
clear all
clear screen

% set matlab output window to long format
format long

%% read generation - frequency

files = rdir('.\best_individuals\Evo_NEAT_run_*_best_gen_*_morph_genome.dat');
[numoffiles, unused] = size(files);

loadidcs = [];
generations = [];
frequencies = [];
readidcs = [];
for i= 1:numoffiles
    fileName = files(i).name;
    frequency = readmorphologyfile(fileName);
    frequencies = [frequencies; frequency];
    values = textscan(fileName, '.\\best_individuals\\Evo_NEAT_run_%d_best_gen_%d_morph_genome.dat');
    %runnum = values{1};
    generations = [generations; values{2}];
    readidcs = [readidcs; i];
end

%sort generations and frequencies by generations
aux_sortmatrix = [generations readidcs];
aux_sortmatrix = sortrows(aux_sortmatrix);
sgenerations = aux_sortmatrix(:,1);
sfrequencies = frequencies(aux_sortmatrix(:,2));

% figure;
% hold on;
% title('Scatter frequency/generation')
% xlabel('generation');
% ylabel('frequency (Hz)');
% a = 1;
% c = linspace(1,10,length(generations));
% scatter(generations, frequencies, a, c);
% hold off;
% 
% figure;
% hold on;
% title('Scatter frequency/generation')
% xlabel('generation');
% ylabel('frequency (Hz)');
% a = 3;
% c = linspace(1,10,length(generations));
% scatter(sgenerations, sfrequencies, a, c, 'filled');
% hold off;
figure;
hold on;
title('plot frequency of best individual per generation')
xlabel('generation');
ylabel('frequency (Hz)');
plot(sgenerations, sfrequencies,'x-');
hold off;

%% read generation - fitnesses
figure;
%hold on;
hold all; % for automatic plot coloring
grid on;
xlabel('generation #')
ylabel('fitness')
title('fitness progress of the best individual')
[gens, individuals, fitnesses] = readoutfile('0_best_individuals_logging.dat');
plot(gens, fitnesses);
hold off;


%% combine generations - frequencies - fitnesses

figure;
hold on;
grid on;
title('Scatter frequency/generation')
xlabel('frequency (Hz)');
ylabel('fitness');
a = 3;
c = linspace(1,10,length(generations));
scatter(sfrequencies, fitnesses, a, c, 'filled');
hold off;

end
%%
function frequencies = readmorphologyfile(fname)
    inputfile = fopen(fname);
    values = textscan(inputfile, '%f');
    frequencies = values{1};
    fclose(inputfile);
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