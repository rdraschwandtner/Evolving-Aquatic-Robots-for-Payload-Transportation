function analysis_freq_amp_phi()
clear all
clear screen
close all

% set matlab output window to long format
format long

limy_ampl = [0 1.6];
ytick_ampl = [0.1:0.2:1.5];
limy_freq = [0 3.1];
ytick_freq = [0.1:0.2:3,3];
limy_ph = [0. 2*pi];
ytick_ph = [0,pi/4,pi/2,3*pi/4,pi,5*pi/4,3*pi/2,7*pi/4,2*pi];
ytick_ph = round(ytick_ph * 100)/100;
%limx = [0:999];
xtick = [0:5:999,999];


set(0,'defaultfigurecolor',[1 1 1])
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(18,'w',func);

light_green = [0 1 0];
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
repmean_frequencies = mean(all_frequencies,2);

stderr =  std(all_frequencies')/sqrt(18);
confmin = repmean_frequencies-(1.96*stderr');
confmax = repmean_frequencies+(1.96*stderr');

upper = max(all_frequencies');
lower = min(all_frequencies');

le2 = ciplot(confmin,confmax,all_generations(:,1),light_green);
le3 = plot(all_generations(:,1),lower,'r--','LineWidth',2);
plot(all_generations(:,1),upper,'r--','LineWidth',2);
le1 = plot(all_generations(:,1), repmean_frequencies,'kd-','MarkerFaceColor','k','LineWidth',2);

breakxaxis([99 990],0.01);
hl = legend([le1 le2 le3],...
    'Mean max fitness','CI max fitness','Range max fitness');
set(hl,'Location','best','Fontsize',10);
hold off;
export_fig('evolve_forward_swimming_after_grasp_freq.pdf','-painters','-pdf');

figure('units','normalized','outerposition',[0 0 1 1]);
hold all;
ylim(limy_ampl);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick_ampl,'Fontsize',10);
title('Max fitness amplitude progress','Fontsize',17);
xlabel('Generation','Fontsize',15);
ylabel('Amplitude','Fontsize',15);
repmean_ampl = mean(all_amplitudes,2);

stderr =  std(all_amplitudes')/sqrt(18);
confmin = repmean_ampl-(1.96*stderr');
confmax = repmean_ampl+(1.96*stderr');

upper = max(all_amplitudes');
lower = min(all_amplitudes');

le2 = ciplot(confmin,confmax,all_generations(:,1),light_green);
le3 = plot(all_generations(:,1),lower,'r--','LineWidth',2);
plot(all_generations(:,1),upper,'r--','LineWidth',2);
le1 = plot(all_generations(:,1), repmean_ampl,'kd-','MarkerFaceColor','k','LineWidth',2);

breakxaxis([99 990],0.01);
hl = legend([le1 le2 le3],...
    'Mean max fitness','CI max fitness','Range max fitness');
set(hl,'Location','best','Fontsize',10);
hold off;
export_fig('evolve_forward_swimming_after_grasp_amp.pdf','-painters','-pdf');

figure('units','normalized','outerposition',[0 0 1 1]);
hold all;
ylim(limy_ph);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick_ph,'Fontsize',10);
title('Max fitness phaseshift progress','Fontsize',17);
xlabel('Generation','Fontsize',15);
ylabel('Phase shift (rad)','Fontsize',15);

repmean_phss = mean(all_phaseshifts,2);

stderr =  std(all_phaseshifts')/sqrt(18);
confmin = repmean_phss-(1.96*stderr');
confmax = repmean_phss+(1.96*stderr');

upper = max(all_phaseshifts');
lower = min(all_phaseshifts');

le2 = ciplot(confmin,confmax,all_generations(:,1),light_green);
le3 = plot(all_generations(:,1),lower,'r--','LineWidth',2);
plot(all_generations(:,1),upper,'r--','LineWidth',2);
le1 = plot(all_generations(:,1), repmean_phss,'kd-','MarkerFaceColor','k','LineWidth',2);

breakxaxis([99 990],0.01);
hl = legend([le1 le2 le3],...
    'Mean max fitness','CI max fitness','Range max fitness');
set(hl,'Location','best','Fontsize',10);
hold off;
export_fig('evolve_forward_swimming_after_grasp_phss.pdf','-painters','-pdf');

%% show params
func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(18,'w',func);

lastgen_freq = all_frequencies(end,:);
lastgen_ampl = all_amplitudes(end,:);
lastgen_phss = all_phaseshifts(end,:);

figure('units','normalized','outerposition',[0 0 1 1]); %fullscreen figure;);
subplot(1,2,1);
hold on;
ylim(limy_ampl);
xlim(limy_freq);
for replicatenum = 0:17
    plot(lastgen_phss(replicatenum+1),lastgen_ampl(replicatenum+1),...
        'o','MarkerEdgeColor',colors(replicatenum+1,:),...
        'MarkerFaceColor',colors(replicatenum+1,:),'MarkerSize',5);
end
xlabel('Frequency (Hz)','Fontsize',15);
ylabel('Amplitude','Fontsize',15);
set(gca,'XTick',ytick_freq,'YTick',ytick_ampl,'Fontsize',10);
hold off;
subplot(1,2,2);
hold on;
ylim(limy_ampl);
xlim(limy_ph);
legendstr = {}
for replicatenum = 0:17
    plot(lastgen_phss(replicatenum+1),lastgen_ampl(replicatenum+1),...
        'o','MarkerEdgeColor',colors(replicatenum+1,:),...
        'MarkerFaceColor',colors(replicatenum+1,:),'MarkerSize',5);
    legendstr{replicatenum+1} = sprintf('%d',replicatenum);
end
xlabel('Phase shift (rad)','Fontsize',15);
set(gca,'XTick',ytick_ph,'YTick',ytick_ampl,'YTicklabel',[],'Fontsize',10);
legend(legendstr,'location','best','Orientation','horizontal');
hold off;
disp('adjust legend and hit enter');
pause;
export_fig('evolve_forward_swimming_after_grasp_params.pdf','-painters','-pdf', '-p5');

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
