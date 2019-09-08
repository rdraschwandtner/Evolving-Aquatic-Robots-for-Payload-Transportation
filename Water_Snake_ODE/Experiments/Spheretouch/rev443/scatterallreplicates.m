

for replicatenum = 0:17
    
    loadidcs = [];
    generations = [];
    frequencies = [];
    readidcs = [];
    for i= 1:numoffiles
        fileName = files(i).name;
        frequency = readmorphologyfile(fileName);
        frequencies = [frequencies; frequency];
        values = textscan(fileName, '.\\%d\\best_individuals\\Evo_NEAT_run_%d_best_gen_%d_morph_genome.dat');
        %runnum = values{1};
        generations = [generations; values{3}];
        readidcs = [readidcs; i];
    end

    %sort generations and frequencies by generations
    aux_sortmatrix = [generations readidcs];
    aux_sortmatrix = sortrows(aux_sortmatrix);
    sgenerations = aux_sortmatrix(:,1);
    sfrequencies = frequencies(aux_sortmatrix(:,2));
    
    
    %hold on;
    hold all; % for automatic plot coloring
    grid on;
    xlabel('generation #')
    ylabel('fitness')
    title('fitness progress of the best individual')
    filestrbestindiv = sprintf('.\\%d\\%d_best_individuals_logging.dat',replicatenum,replicatenum);
    [gens, individuals, fitnesses] = readoutfile(filestrbestindiv);
    plot(gens, fitnesses);
    hold off;
end