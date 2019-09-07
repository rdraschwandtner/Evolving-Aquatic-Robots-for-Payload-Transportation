function analysis_lastpop()
    clear all
    clear screen
    %
    %close all

    % set matlab output window to long format
    format long

    % amplitudes
    %       joint1 .. joint#
    % indiv
    lastgen_ampl = [];
    lastgen_freq = [];
    lastgen_phi = [];
    legendstrs = {};
    for repl_num = 0:17
        filestr = sprintf('./%d/best_individuals/Evo_NEAT_run_%d_best_gen_999_morph_genome.dat', repl_num,repl_num);
        [ampls, freqs, phis] = readparams(filestr);
        lastgen_ampl = [lastgen_ampl; ampls];
        lastgen_freq = [lastgen_freq; freqs];
        lastgen_phi = [lastgen_phi; phis];
        legendstrs = [legendstrs; sprintf('repl%d',repl_num)];
    end
    
    ampl_mean = mean(lastgen_ampl);
    freq_mean = mean(lastgen_freq);
    phi_mean = mean(lastgen_phi);
    
    figure;
    hold on;
    grid on;
    plot(0:length(lastgen_ampl)-1, lastgen_ampl, '*');
    plot(0:length(lastgen_ampl)-1, ones(length(lastgen_ampl))*ampl_mean, 'g-');
    title('Amplitudes of last generation');
    xlabel('replicate number');
    ylabel('amplitude (rad)');
    legend('values','mean');
    hold off;
    
    figure;
    hold on;
    grid on;
    plot(0:length(lastgen_freq)-1, lastgen_freq, '*');
    plot(0:length(lastgen_freq)-1, ones(length(lastgen_freq))*freq_mean, 'g-');
    title('Frequencies of last generation');
    xlabel('replicate number');
    ylabel('frequency (Hz)');
    legend('values','mean');
    hold off;
    
    figure;
    hold on;
    grid on;
    plot(0:length(lastgen_phi)-1, lastgen_phi, '*');
    plot(0:length(lastgen_phi)-1, ones(length(lastgen_phi))*phi_mean, 'g-');
    title('Amplitudes of last generation');
    xlabel('replicate number');
    ylabel('phaseshift (rad)');
    hold off;

end

function [ampl, freq, phi] = readparams(fname)
    inputfile = fopen(fname);
    values = textscan(inputfile, '[%f, %f, %f]');
    ampl = values{1};
    freq = values{2};
    phi = values{3};
    fclose(inputfile);
end
