function jointanalysis(replicatenum)

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
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6]
    sval_mat = sortrows(val_mat, 1)
    stime = sval_mat(:,1);
    sjoint1angle = sval_mat(:,3);
    sjoint2angle = sval_mat(:,5);
    sjoint3angle = sval_mat(:,7);
    sjoint4angle = sval_mat(:,9);
    sjoint5angle = sval_mat(:,11);
    sjoint6angle = sval_mat(:,13);
    sjoint7angle = sval_mat(:,15);
%    figure;
%    hold all;
%    title('jointangle course');
%    ylabel('joint angle (rad)');
%    xlabel('time');
%     plot(stime,sjoint1angle);
%    plot(stime,sjoint2angle);
%     plot(stime,sjoint3angle);
%     plot(stime,sjoint4angle);
%     plot(stime,sjoint5angle);
%     plot(stime,sjoint6angle);
%     plot(stime,sjoint7angle);
%    legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
%    hold off;
    
%     % http://www.mathworks.com/help/matlab/examples/fft-for-spectral-analysis.html
%     numofsamples = length(stime);
%     Y2 = fft(sjoint2angle,numofsamples);
%     Pyy2 = Y2.*conj(Y2)/numofsamples;
%     % http://stackoverflow.com/questions/10758315/understanding-matlab-fft-example
%     faxis = linspace(0,1,int8(numofsamples/2)+1); % show only relevant frequencies before nyqhist freq    
    
%     T = 0.02; % sample time
%     Fs = 1/T; % sample frequency
%     L = length(stime)
%     % timevector .. stime
%     NFFT = 2^nextpow2(L); % Next power of 2 from length of y
%     Y = fft(sjoint2angle,NFFT)/L;
%     f = Fs/2*linspace(0,1,NFFT/2+1);
%     plot(f,2*abs(Y(1:NFFT/2+1)))
%     title('Single-Sided Amplitude Spectrum of y(t)')
%     xlabel('Frequency (Hz)')
%     ylabel('|Y(f)|')

    
%     % find the most dominant frequency
%     maxmagnitude = max(2*abs(Y(1:NFFT/2+1)));
%     fmaxidx = find(2*abs(Y(1:NFFT/2+1)) == maxmagnitude);
%     fmax = f(fmaxidx);
%     %fmax = 0.16

    fjoint1 = principalfrequency(stime, sjoint1angle, 0.02);
    fjoint2 = principalfrequency(stime, sjoint2angle, 0.02);
    fjoint3 = principalfrequency(stime, sjoint3angle, 0.02);
    fjoint4 = principalfrequency(stime, sjoint4angle, 0.02);
    fjoint5 = principalfrequency(stime, sjoint5angle, 0.02);
    fjoint6 = principalfrequency(stime, sjoint6angle, 0.02);
    fjoint7 = principalfrequency(stime, sjoint7angle, 0.02);

    titlestr = sprintf('jointana rep: %d', replicatenum);
    % make figure fullscreen an 
    figure('units','normalized','outerposition',[0 0 1 1], 'Name', titlestr);
    subplot(3,3,1);
    hold all;
    title('jointangle 1');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint1angle);
    a = max(sjoint1angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint1*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');
    hold off;
    
    subplot(3,3,2);
    hold all
    title('jointangle 2');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint2angle);
    a = max(sjoint2angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint2*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,3);
    hold all;
    title('jointangle 3');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint3angle);
    a = max(sjoint3angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint3*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,4);
    hold all;
    title('jointangle 4');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint4angle);
    a = max(sjoint4angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint4*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
    subplot(3,3,5);
    hold all;
    title('jointangle 5');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint5angle);
    a = max(sjoint5angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint5*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;

    subplot(3,3,6);
    hold all;
    title('jointangle 6');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint6angle);
    a = max(sjoint6angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint6*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;

    subplot(3,3,7);
    hold all;
    title('jointangle 7');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint7angle);
    a = max(sjoint4angle); % normalize to the maximum of the original signal
    x = a * sin(2*pi*fjoint7*stime);
    plot(stime, x);
    %legend('orig signal','approx signal');    
    hold off;
    
end

function frequency = principalfrequency(time, signal, sampletime)
    sampletime = 0.02; % sample time
    Fs = 1/sampletime; % sample frequency
    L = length(time)
    % timevector .. stime
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
    Y = fft(signal,NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);
    figure;
    plot(f,2*abs(Y(1:NFFT/2+1)))
    title('Single-Sided Amplitude Spectrum of y(t)')
    xlabel('Frequency (Hz)')
    ylabel('|Y(f)|')
    grid on;

    
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