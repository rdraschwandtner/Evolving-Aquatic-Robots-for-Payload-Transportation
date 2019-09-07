function analysis_freq_amp_phi()
clear all
clear screen

% set matlab output window to long format
format long

%       rep1 | rep2 | rep3
% gen1
% gen2
all_generations = [];
all_amplitudes = [];
all_frequencies = [];
all_phaseshifts = [];

%% read generation - frequency
for replicatenum = 0:17
    filestr = sprintf('.\\%d\\best_individuals\\Evo_NEAT_run_%d_best_gen_*_morph_genome.dat',replicatenum,replicatenum);
    files = rdir(filestr);
    [numoffiles, unused] = size(files);

    generations = [];
    frequencies = [];
    amplitudes = [];
    phaseshifts = [];
    readidcs = [];
    for i= 1:numoffiles
        fileName = files(i).name;
        [amplitude, frequency, phaseshift] = readmorphologyfile(fileName);
        amplitudes = [amplitudes; amplitude];
        frequencies = [frequencies; frequency];
        phaseshifts = [phaseshifts; phaseshift];
        values = textscan(fileName, '.\\%d\\best_individuals\\Evo_NEAT_run_%d_best_gen_%d_morph_genome.dat');
        
        generations = [generations; values{3}];
        readidcs = [readidcs; i];
    end

    %sort generations and frequencies by generations
    aux_sortmatrix = [generations readidcs];
    aux_sortmatrix = sortrows(aux_sortmatrix);
    sgenerations = aux_sortmatrix(:,1);
    sfrequencies = frequencies(aux_sortmatrix(:,2));

%     titlestr = sprintf('fit rep: %d', replicatenum);
%     % make figure fullscreen an
%     figure('units','normalized','outerposition',[0 0 1 1], 'Name', titlestr);
%     subplot(3,1,1);
%     hold on;
%     title('plot frequency of best individual per generation')
%     xlabel('generation');
%     ylabel('frequency (Hz)');
%     plot(sgenerations, sfrequencies);
%     hold off;
%     
%     %%
     samplitudes = amplitudes(aux_sortmatrix(:,2));
%     
%     subplot(3,1,2);
%     hold on;
%     title('plot amplitude of best individual per generation')
%     xlabel('generation');
%     ylabel('amplitude');
%     plot(sgenerations, samplitudes);
%     hold off;    
%     
     sphaseshifts = phaseshifts(aux_sortmatrix(:,2));
%     subplot(3,1,3);
%     hold on;
%     title('plot phaseshift of best individual per generation')
%     xlabel('generation');
%     ylabel('phaseshift');
%     plot(sgenerations, sphaseshifts);
%     hold off;
%     
%     pause;
    
    
    all_generations = [all_generations sgenerations];
    all_frequencies = [all_frequencies sfrequencies];
    all_amplitudes = [all_amplitudes samplitudes];
    all_phaseshifts = [all_phaseshifts sphaseshifts];    
end

figure(1);
hold all;
grid on;
title('Best individual frequency progress over generations');
xlabel('Generation');
ylabel('Frequency (Hz)');

figure(2);
hold all;
grid on;
title('Best individual amplitude progress over generations');
xlabel('Generation');
ylabel('Amplitude (rad)');

figure(3);
hold all;
grid on;
title('Best individual phaseshift progress over generations');
xlabel('Generation');
ylabel('Phase shift (rad)');

legendstr = {};
for rep_num = 0:17
    figure(1);  
    plot(all_generations(:,rep_num+1), all_frequencies(:,rep_num+1));
    figure(2);
    plot(all_generations(:,rep_num+1), all_amplitudes(:,rep_num+1));
    figure(3);
    plot(all_generations(:,rep_num+1), all_phaseshifts(:,rep_num+1));
    
    legendstr{rep_num+1} = sprintf('repnum %d', rep_num);
end

figure(1);
legend(legendstr);

figure(2);
legend(legendstr);

figure(3);
legend(legendstr);


end
%%
function [amplitude, frequency, phaseshift] = readmorphologyfile(fname)
    inputfile = fopen(fname);
    values = textscan(inputfile, '[%f, %f, %f]');
    amplitude = values{1};
    frequency = values{2};
    phaseshift = values{3};
    fclose(inputfile);
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
