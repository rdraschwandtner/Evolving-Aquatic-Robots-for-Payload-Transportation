function analysis_lastpop()
    clear all
    clear screen
    close all

    % set matlab output window to long format
    format long

    % amplitudes
    %       joint1 .. joint#
    % indiv
    lastgen_ampl = [];
    lastgen_freq = [];
    lastgen_phi = [];
    
    files = rdir('.\**\snake_gen199_bestindvidx*.json')
    [numoffiles, unused] = size(files);
    for i= 1:numoffiles
        files(i).name
        [ampls, freqs, phis] = readparams(files(i).name);
        lastgen_ampl = [lastgen_ampl; ampls];
        lastgen_freq = [lastgen_freq; freqs];
        lastgen_phi = [lastgen_phi; phis];
    end
    
    mean(lastgen_ampl)
   
    figure;
    boxplot(lastgen_ampl .* 180/pi, 'labels', {'1','2','3','4','5','6','7'})
    title('Amplitude distribution per joint (best individual of last generation, 18 generations)')
    xlabel('Joint number')
    ylabel('Amplitude (deg)')
    
    figure;
    boxplot(lastgen_freq, 'labels', {'1','2','3','4','5','6','7'})
    title('Frequency distribution per joint (best individual of last generation, 18 generations)')
    xlabel('Joint number')
    ylabel('Frequency (Hz)')
    
    figure;
    boxplot(lastgen_phi .* 180/pi, 'labels', {'1','2','3','4','5','6','7'})
    title('Phi distribution per joint (best individual of last generation, 18 generations)')
    xlabel('Joint number')
    ylabel('Phi (deg)')

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
