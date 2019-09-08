function MA_analysis_lastpop()
    clear all
    clear screen
    %
    %close all

    % set matlab output window to long format
    format long
    
    set(0,'defaultfigurecolor',[1 1 1]);

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
        files(i).name
        [ampls, freqs, phis] = readparams(files(i).name);
        lastgen_ampl = [lastgen_ampl; ampls];
        lastgen_freq = [lastgen_freq; freqs];
        lastgen_phi = [lastgen_phi; phis];
        legendstrs = [legendstrs; files(i).name];
    end
       
    figure('units','normalized','outerposition',[1/2 1/2 1/2 1/2]);
    boxplot(lastgen_ampl, 'labels', {'0','1','2','3','4','5','6'});
    set(gca,'YTick',[0:0.2:2]);
    set(gca,'Fontsize',15);   
    h = findobj(gca,'Type','text')
    set(h,'Fontsize',15)
    title('Max fitness amplitude distribution per joint','Fontsize',20);
    xlabel('Joint number','Fontsize',18);
    ylabel('Amplitude','Fontsize',18);
    afname = exportfname('Adistr','gen999');
    export_fig(afname,'-painters','-pdf', '-p5');
    
    figure('units','normalized','outerposition',[1/2 1/2 1/2 1/2]);
    boxplot(lastgen_freq, 'labels', {'0','1','2','3','4','5','6'});
    set(gca,'YTick',[0:0.4:4]);
    set(gca,'Fontsize',15);
    h = findobj(gca,'Type','text')
    set(h,'Fontsize',15)
    title('Max fitness frequency distribution per joint','Fontsize',20);
    xlabel('Joint number','Fontsize',18);
    ylabel('Frequency (Hz)','Fontsize',18);
    ffname = exportfname('Fdistr','gen999');
    export_fig(ffname,'-painters','-pdf', '-p5');
    
    figure('units','normalized','outerposition',[1/2 1/2 1/2 1/2]);
    boxplot(lastgen_phi, 'labels', {'0','1','2','3','4','5','6'});
    ytick_ph = [0,pi/4,pi/2,3*pi/4,pi,5*pi/4,3*pi/2,7*pi/4,2*pi];
    ytick_ph = round(ytick_ph * 100)/100;
    set(gca,'Fontsize',15);
    set(gca,'YTick',ytick_ph);
    h = findobj(gca,'Type','text')
    set(h,'Fontsize',15)
    title('Max phase shift distribution per joint','Fontsize',20);
    xlabel('Joint number','Fontsize',18);
    ylabel('Phase shift (rad)','Fontsize',18);
    pfname = exportfname('Pdistr','gen999');
    export_fig(pfname,'-painters','-pdf', '-p5');
    
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

function fname=exportfname(prefix,postfix)
    curdir = pwd;
    dirs = textscan(curdir,'%s','Delimiter','\\');
    dirstr = dirs{1};
    fname = strcat(prefix,char(dirstr(end-2)),'_',char(dirstr(end-1)),postfix);
end