
function [upper,lower,avg,gens] = analysis()
clear all
clear screen
close all

limx = [-1 1000];
limy = [-1 25];
ytick = [0:2:24];
xtick = [0:5:999,999];

set(0,'defaultfigurecolor',[1 1 1])



[upper1,lower1,avg1,gens1,confmin,confmax] = analyseBestInidividual();
[upper2,lower2,avg2,gens2,confmin2,confmax2] = analyseAvgInidividual();
lightblue = [0.5  0.5  1];
lightgrey = [0.8  0.8  0.8];
figure('units','normalized','outerposition',[0 0 1 1]); %fullscreen figure
hold on;

ylim(limy);
xlim(limx);
ax = gca;
set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',10);
xlabel('Generation','Fontsize',15);
ylabel('Fitness','Fontsize',15);
title('Fitness progress','Fontsize',17)
%grid on;
%ciplot(lower1,upper1,gens1,lightblue);
le2 = ciplot(confmin,confmax,gens1,lightblue);
le3 = plot(gens1,lower1,'r--','LineWidth',2);
plot(gens1,upper1,'r--','LineWidth',2);
le1 = plot(gens1, avg1,'kd-','MarkerFaceColor','k','LineWidth',2);


%ciplot(lower2,upper2,gens2,lightgrey);
le5 = ciplot(confmin2,confmax2,gens2,lightgrey);
le6 = plot(gens2,lower2,'r--','LineWidth',2);
plot(gens2,upper2,'r--','LineWidth',2)
le4 = plot(gens2, avg2,'ko-','MarkerFaceColor','k','LineWidth',2);

breakxaxis([39 990],0.01);
hl = legend([le1 le2 le3 le4 le5 le6], ...
    'Mean max fitness','CI max fitness','Range max fitness', ...
    'Mean avg fitness','CI avg fitness','Range avg fitness');
set(hl,'Location','best','Fontsize',10);

hold off;
export_fig('evolve_folded_forward_swimming_rev356.pdf','-painters','-pdf');
end


function [upper,lower,avg,gens,confmin,confmax] = analyseAvgInidividual()
    upper = zeros(1000,1); %assuming 1000 generations
    lower = ones(1000,1)*1000000000;
    amp = zeros(1000,1);
    allavgfitnesses = [];
    number_of_replicates = 18;
    for replicatenum = 0:number_of_replicates-1
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

        %plot(firstgen:lastgen, avgfitpergen);
        upper(upper<avgfitpergen')=avgfitpergen(avgfitpergen'>upper)';
        lower(lower>avgfitpergen')=avgfitpergen(avgfitpergen'<lower)';
        amp = amp + avgfitpergen';
        allavgfitnesses = [allavgfitnesses;avgfitpergen]; 
    end
    gens = 0:lastgen;
    avg = amp./number_of_replicates;
    stderr =  std(allavgfitnesses)/sqrt(number_of_replicates);
    confmin = avg-(1.96*stderr');
    confmax = avg+(1.96*stderr');
end

function [upper,lower,avg,gens,confmin,confmax] = analyseBestInidividual()
    upper = zeros(1000,1); %assuming 1000 generations
    lower = ones(1000,1)*1000000000;
    amp = zeros(1000,1);
    %legendstr = {};
    gens = [];
    allfitnesses = [];
    number_of_replicates = 18;
    for replicatenum = 0:number_of_replicates-1
        bestinindivfile = sprintf('./%d/%d_best_individuals_logging.dat', replicatenum, replicatenum);
        [gens, individuals, fitnesses] = readoutfile(bestinindivfile);
        %plot(gens, fitnesses);
        upper(upper<fitnesses)=fitnesses(fitnesses>upper);
        lower(lower>fitnesses)=fitnesses(fitnesses<lower);
        amp = amp + fitnesses;
        allfitnesses = [allfitnesses,fitnesses];
        %legendstr{replicatenum+1} = sprintf('%d', replicatenum);
    end
    avg=amp./number_of_replicates;
    stderr =  std(allfitnesses')/sqrt(number_of_replicates);
    confmin = avg-(1.96*stderr');
    confmax = avg+(1.96*stderr');
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
    
    values = textscan(inputfile, '%d;%d;(%f,)', 'headerLines', Headerlines);
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{3};
end