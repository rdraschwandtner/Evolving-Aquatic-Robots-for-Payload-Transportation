function analysis()
    close all;
    %set(0,'defaultfigurecolor',[1 1 1])
    %func = @(x) colorspace('RGB->Lab',x);
    %colors = distinguishable_colors(18,'w',func);

    lastgen = 199;
    firstgen = 0;
    
    avgfitpergen = [];
    maxfitpergen = [];
    [gens, ind, fitnesses] = readoutfile('0_fitnesses.dat');
    for gennum= firstgen:lastgen
        idcs = find(gens==gennum);
        % assumung gennum starts with 0
        avgfitpergen(gennum+1) = mean(fitnesses(idcs));
        maxfitpergen(gennum+1) = max(fitnesses(idcs));
    end
    
    figure;
    hold on;
    plot(firstgen:lastgen, avgfitpergen, 'b');
    plot(firstgen:lastgen, maxfitpergen, 'k');
    legend('avg', 'max');
    hold off;
end



function [generation,individuals,fitnesses] = readoutfile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile, '%d,%d,%d,%d,%d,%f','HeaderLines',Headerlines);
    fclose(inputfile);
    generation = values{1};
    individuals = values{2};
    fitnesses = values{6}; 
end