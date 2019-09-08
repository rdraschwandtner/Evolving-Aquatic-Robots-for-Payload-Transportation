function analysis_lastpop()
    clear all
    clear screen
    %
    close all

    % set matlab output window to long format
    format long

    % amplitudes
    %       joint1 .. joint#
    % indiv
    lastgen_ampl = [];
    lastgen_freq = [];
    lastgen_phi = [];
    
    files = rdir('.\**\snake_gen999_bestindvidx*.json')
    [numoffiles, unused] = size(files);
    legendstrs = {};
    for i= 1:numoffiles
        % avoid replicate 13 because it has an ODE error
        if i == 13
            continue;
        end
        files(i).name
        [ampls, freqs, phis] = readparams(files(i).name);
        lastgen_ampl = [lastgen_ampl; ampls];
        lastgen_freq = [lastgen_freq; freqs];
        lastgen_phi = [lastgen_phi; phis];
        legendstrs = [legendstrs; files(i).name];
    end
    
    mean(lastgen_ampl)
   
    figure;
    boxplot(lastgen_ampl .* 180/pi, 'labels', {'1','2','3','4','5','6','7'})
    title('Amplitude distribution per joint (best individual of last generation from 17 replicates)')
    xlabel('Joint number')
    ylabel('Amplitude (deg)')
    
    figure;
    boxplot(lastgen_freq, 'labels', {'1','2','3','4','5','6','7'})
    title('Frequency distribution per joint (best individual of last generation from 17 replicates)')
    xlabel('Joint number')
    ylabel('Frequency (Hz)')
    
    figure;
    boxplot(lastgen_phi .* 180/pi, 'labels', {'1','2','3','4','5','6','7'})
    title('Phi distribution per joint (best individual of last generation from 17 replicates)')
    xlabel('Joint number')
    ylabel('Phi (deg)')
    
    figure;
    hold all;
    for i=1:numoffiles -1 % -1 .. replicate 13 error
        plot(1:7, lastgen_freq(i,:),'--x');
    end
    xlabel('Joint number')
    ylabel('Frequency (HZ)')
    legend(legendstrs);
    %grid on;
    hold off;
    
    figure;
    hold all;
    for i=1:numoffiles -1 % -1 .. replicate 13 error
        plot(1:7, lastgen_ampl(i,:),'--x');
    end
    xlabel('Joint number')
    ylabel('Frequency (HZ)')
    legend(legendstrs);
    %grid on;
    hold off;
    
    figure;
    hold all;
    for i=1:numoffiles -1 % -1 .. replicate 13 error
        plot(1:7, lastgen_phi(i,:),'--x');
    end
    xlabel('Joint number')
    ylabel('Frequency (HZ)')
    legend(legendstrs);
    %grid on;
    hold off;
end

function [ampls, freqs, phis] = readparams(fname)
    % taken from http://www.mathworks.com/matlabcentral/fileexchange/42236-parse-json-text/content/example/html/usage.html#7
    fid = fopen(fname);
    raw = fread(fid,inf);
    str = char(raw');
    fclose(fid);

    data = JSON.parse(str);
    %
    ampls=[];
    freqs=[];
    phis=[];
    % iterate through all joints
    for i=1:7
        % data: [genome params], [jointnum], [ampl,freq,phi]
        ampl = data{1}{i}{1};
        freq = data{1}{i}{2};
        phi = data{1}{i}{3};
        ampls=[ampls,ampl];
        freqs=[freqs,freq];
        phis=[phi,phis];
    end
end
