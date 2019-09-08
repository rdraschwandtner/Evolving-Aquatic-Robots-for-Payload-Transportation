function MA_jointanalysis()
    lastgen_freq = [];
    for replicatenum = 0:17
        lastgen_freq = [lastgen_freq;jointanalysis(replicatenum,0)];
    end
    
    figure('units','normalized','outerposition',[1/2 1/2 1/2 1/2]);
    boxplot(lastgen_freq, 'labels', {'0','1','2','3','4','5','6'});
    set(gca,'YTick',[0:0.4:4]);
    ylim([0 4]);
    set(gca,'Fontsize',10);
    title('Max fitness frequency distribution per joint (generation 999)','Fontsize',17);
    xlabel('Joint number','Fontsize',15);
    ylabel('Frequency (Hz)','Fontsize',15);
    ffname = exportfname('Fdistr','gen999');
    export_fig(ffname,'-painters','-pdf', '-p5');
    
    % plot input frequency    
    [upper,lower,avg,gens,confmin,confmax] = calc_input_frequencies();    
    light_green = [0 1 0];
    figure('units','normalized','outerposition',[0 0 1 1]); %fullscreen figure;);
    hold all;
    %ylim(limy_freq);

    set(gca,'XTick',[0:20:999,999],'YTick',[0:0.5:4]);
    set(gca,'Fontsize',15);
    title('NN input frequency progress','Fontsize',22);
    xlabel('Generation','Fontsize',20);
    ylabel('Input frequency (Hz)','Fontsize',20);

    le2 = ciplot(confmin,confmax,gens,light_green);
    le3 = plot(gens,lower,'r--','LineWidth',2);
    plot(gens,upper,'r--','LineWidth',2);
    le1 = plot(gens([1:10:end end]), avg([1:10:end end]),'kd-','MarkerFaceColor','k','LineWidth',2);

    breakxaxis([399 990],0.01);
    hl = legend([le1 le2 le3],...
        'Mean frequency','CI frequency','Range frequency');
    set(hl,'Location','best','Fontsize',15);
    hold off;
    ffname = exportfname('inputfreqprogress',[]);
    export_fig(ffname,'-painters','-pdf', '-p5');

    jointanalysis(0,1)
    disp('adjust legend');
    pause
    fname = exportfname('jointanglecomp',[]);
    export_fig(fname,'-painters','-pdf', '-p5');
end

function [upper,lower,avg,gens,confmin,confmax] = calc_input_frequencies()
    all_freqs = [];
    number_of_replicates = 18;
    for replicatenum = 0:number_of_replicates-1
        filestr = sprintf('.\\%d\\best_individuals\\Evo_NEAT_run_%d_best_gen_*_morph_genome.dat',replicatenum,replicatenum);
        files = rdir(filestr);
        [numoffiles, ~] = size(files);

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
        all_freqs = [all_freqs; sfrequencies']; 
    end
    upper = max(all_freqs);
    lower = min(all_freqs);
    gens = 0:sgenerations(end);
    avg = mean(all_freqs);
    stderr =  std(all_freqs)/sqrt(number_of_replicates);
    confmin = avg-(1.96*stderr);
    confmax = avg+(1.96*stderr);    
end


function freqs = jointanalysis(replicatenum, printflag)

    if nargin < 1
        replicatenum = 0
    end
    % set matlab output window to long format
    %format long

    jointfilestr = sprintf('.\\%d\\validator_logging\\joint_angles\\rep%d_gen999_joint_angles.dat', replicatenum, replicatenum);    
    [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6] = ...
    readjointanglefile(jointfilestr);
    val_mat = [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6];
    sval_mat = sortrows(val_mat, 1);
    stime = sval_mat(:,1);
    sjoint1angle = sval_mat(:,3);
    sjoint2angle = sval_mat(:,5);
    sjoint3angle = sval_mat(:,7);
    sjoint4angle = sval_mat(:,9);
    sjoint5angle = sval_mat(:,11);
    sjoint6angle = sval_mat(:,13);
    sjoint7angle = sval_mat(:,15);

    fjoint1 = principalfrequency(stime, sjoint1angle, 0.02, -1);
    fjoint2 = principalfrequency(stime, sjoint2angle, 0.02, -1);
    fjoint3 = principalfrequency(stime, sjoint3angle, 0.02, -1);
    fjoint4 = principalfrequency(stime, sjoint4angle, 0.02, -1);
    fjoint5 = principalfrequency(stime, sjoint5angle, 0.02, -1);
    fjoint6 = principalfrequency(stime, sjoint6angle, 0.02, -1);
    fjoint7 = principalfrequency(stime, sjoint7angle, 0.02, -1);
   
    freqs = [fjoint1 fjoint2 fjoint3 fjoint4 fjoint5 fjoint6 fjoint7];
    
    if printflag == 1
        plotjointangles(freqs,...
            [sjoint1angle sjoint2angle sjoint3angle sjoint4angle sjoint5angle sjoint6angle sjoint7angle]...
            ,stime);
    end

end

function plotjointangles(princ_freqs, act_jointangles,stime)
    % make figure fullscreen an 
    figure('units','normalized','outerposition',[0 0 1 1]);
    subplot(3,3,1);
    hold all;
    title('Joint 0','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,1));
    a = max(act_jointangles(:,1)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(1)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');
    hold off;
    
    subplot(3,3,2);
    hold all
    title('Joint 1','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,2));
    a = max(act_jointangles(:,2)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(2)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,3);
    hold all;
    title('Joint 2','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,3));
    a = max(act_jointangles(:,3)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(3)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,4);
    hold all;
    title('Joint 3','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,4));
    a = max(act_jointangles(:,4)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(4)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,5);
    hold all;
    title('Joint 4','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,5));
    a = max(act_jointangles(:,5)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(5)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;

    subplot(3,3,6);
    hold all;
    title('Joint 5','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,6));
    a = max(act_jointangles(:,6)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(6)*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;

    subplot(3,3,7);
    hold all;
    title('Joint 6','Fontsize',15);
    ylabel('Joint angle (rad)','Fontsize',12);
    xlabel('Time (s)','Fontsize',12);
    plot(stime,act_jointangles(:,7));
    a = max(act_jointangles(:,7)); % normalize to the maximum of the original signal
    x = a * sin(2*pi*princ_freqs(7)*stime);
    plot(stime, x);
    legend('original signal','a*sin(2*\pi*principalfreq)','Interpreter','latex');
    hold off;
end

function frequency = principalfrequency(time, signal, sampletime, fnum)
    sampletime = 0.02; % sample time
    Fs = 1/sampletime; % sample frequency
    L = length(time);
    % timevector .. stime
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
    Y = fft(signal,NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);
    if fnum >= 0
        subplot(3,3,fnum);
        plot(f,2*abs(Y(1:NFFT/2+1)));
        title('Single-Sided Amplitude Spectrum of y(t)');
        xlabel('Frequency (Hz)');
        ylabel('|Y(f)|');
        grid on;
    end

    
    % find the most dominant frequency
    maxmagnitude = max(2*abs(Y(1:NFFT/2+1)));
    fmaxidx = find(2*abs(Y(1:NFFT/2+1)) == maxmagnitude);
    fmax = f(fmaxidx);
    frequency = fmax;
end

function [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6] = readjointanglefile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f', 'headerLines', Headerlines);
    fclose(inputfile);
    Time = values{1};
    Angle_1_0 = values{2};
    Angle_2_0 = values{3};
    Angle_1_1 = values{4};
    Angle_2_1 = values{5};
    Angle_1_2 = values{6};
    Angle_2_2 = values{7};
    Angle_1_3 = values{8};
    Angle_2_3 = values{9};
    Angle_1_4 = values{10};
    Angle_2_4 = values{11};
    Angle_1_5 = values{12};
    Angle_2_5 = values{13};
    Angle_1_6 = values{14};
    Angle_2_6 = values{15};
end

function fname=exportfname(prefix,postfix)
    curdir = pwd;
    dirs = textscan(curdir,'%s','Delimiter','\\');
    dirstr = dirs{1};
    fname = strcat(prefix,char(dirstr(end-2)),'_',char(dirstr(end-1)),postfix);
end

function frequencies = readmorphologyfile(fname)
    inputfile = fopen(fname);
    values = textscan(inputfile, '%f');
    frequencies = values{1};
    fclose(inputfile);
end