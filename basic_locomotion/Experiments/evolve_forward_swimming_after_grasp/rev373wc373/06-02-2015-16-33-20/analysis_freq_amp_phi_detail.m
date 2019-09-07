function MA_analysis_freq_amp_phi()
clear all
clear screen
close all

% set matlab output window to long format
format long

limy_ampl = [0 1.6];
ytick_ampl = [0.1:0.2:1.5];
limy_freq = [0 3.1];
ytick_freq = [0.1:0.2:3];
limy_ph = [0. 2*pi];
ytick_ph = [0,pi/4,pi/2,3*pi/4,pi,5*pi/4,3*pi/2,7*pi/4,2*pi];
ytick_ph = round(ytick_ph * 100)/100;
%limx = [0:999];
xtick = [0:5:999,999];


set(0,'defaultfigurecolor',[1 1 1])
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(18,'w',func);


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

    samplitudes = amplitudes(aux_sortmatrix(:,2));

    sphaseshifts = phaseshifts(aux_sortmatrix(:,2));

    all_generations = [all_generations sgenerations];
    all_frequencies = [all_frequencies sfrequencies];
    all_amplitudes = [all_amplitudes samplitudes];
    all_phaseshifts = [all_phaseshifts sphaseshifts];    
end

figure('units','normalized','outerposition',[0 0 1 1]); %fullscreen figure;);
hold all;
ylim(limy_freq);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick_freq,'Fontsize',10);
title('Max fitness frequency progress','Fontsize',17);
xlabel('Generation','Fontsize',15);
ylabel('Frequency (Hz)','Fontsize',15);
legendstr = {};
for rep_num = 0:17
    plot(all_generations(:,rep_num+1), all_frequencies(:,rep_num+1),'color', colors(rep_num+1,:));
    legendstr{rep_num+1} = sprintf('%d', rep_num);
end
legend(legendstr,'location','eastoutside');
breakxaxis([99 990],0.01);
hold off;

figure('units','normalized','outerposition',[0 0 1 1]);
hold all;
ylim(limy_ampl);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick_ampl,'Fontsize',10);
title('Max fitness amplitude progress','Fontsize',17);
xlabel('Generation','Fontsize',15);
ylabel('Amplitude','Fontsize',15);
legendstr = {};
for rep_num = 0:17
    plot(all_generations(:,rep_num+1), all_amplitudes(:,rep_num+1),'color', colors(rep_num+1,:));
    legendstr{rep_num+1} = sprintf('%d', rep_num);
end
legend(legendstr,'location','eastoutside');
breakxaxis([99 990],0.01);
hold off;


figure('units','normalized','outerposition',[0 0 1 1]);
hold all;
ylim(limy_ph);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick_ph,'Fontsize',10);
title('Max fitness phaseshift progress','Fontsize',17);
xlabel('Generation','Fontsize',15);
ylabel('Phase shift (rad)','Fontsize',15);
legendstr = {};
for rep_num = 0:17
    plot(all_generations(:,rep_num+1), all_phaseshifts(:,rep_num+1),'color', colors(rep_num+1,:));
    legendstr{rep_num+1} = sprintf('%d', rep_num);
end
legend(legendstr,'location','eastoutside');
breakxaxis([99 990],0.01);
hold off;



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
