function jointanalysis7()

    close all;
    jointfilestr = sprintf('validator_logging\\joint_angles\\_joint_angles.dat');    
    [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6] = ...
    readjointanglefile(jointfilestr);

    val_mat = [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6];

    sval_mat = sortrows(val_mat, 1);
    stime = sval_mat(:,1);
    sjoint1angle = invscale(-pi,pi,sval_mat(:,3));
    sjoint2angle = invscale(-pi,pi,sval_mat(:,5));
    sjoint3angle = invscale(-pi,pi,sval_mat(:,7));
    sjoint4angle = invscale(-pi,pi,sval_mat(:,9));
    sjoint5angle = invscale(-pi,pi,sval_mat(:,11));
    sjoint6angle = invscale(-pi,pi,sval_mat(:,13));
    sjoint7angle = invscale(-pi,pi,sval_mat(:,15));
    
    
    
    
    [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7] = readexpectedjointangle('expected_joint_angles.out')

    eval_mat = [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7];

    seval_mat = sortrows(eval_mat, 1);
    setime = seval_mat(:,1);
    %assert(isequal(stime,setime)) % attention: these are floating point values!
    
    sejoint1angle = invscale(-pi,pi,seval_mat(:,2));
    sejoint2angle = invscale(-pi,pi,seval_mat(:,3));
    sejoint3angle = invscale(-pi,pi,seval_mat(:,4));
    sejoint4angle = invscale(-pi,pi,seval_mat(:,5));
    sejoint5angle = invscale(-pi,pi,seval_mat(:,6));
    sejoint6angle = invscale(-pi,pi,seval_mat(:,7));
    sejoint7angle = invscale(-pi,pi,seval_mat(:,8)); 
        
    
    vname=@(x) inputname(1); %http://www.mathworks.com/matlabcentral/newsreader/view_thread/251347
    
    
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint1angle));
    createjointangleplot(titlestr, stime,sjoint1angle, sejoint1angle);
    legend('current angle', 'expected angle')
    grid on;
    
  
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint2angle));
    createjointangleplot(titlestr, stime,sjoint2angle, sejoint2angle);
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint3angle));
    createjointangleplot(titlestr, stime,sjoint3angle, sejoint3angle);
    legend('current angle', 'expected angle')
    grid on;
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint4angle));
    createjointangleplot(titlestr, stime,sjoint4angle, sejoint4angle);
    legend('current angle', 'expected angle')
    grid on;
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint5angle));
    createjointangleplot(titlestr, stime,sjoint5angle, sejoint5angle);
    legend('current angle', 'expected angle')
    grid on;
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint6angle));
    createjointangleplot(titlestr, stime,sjoint6angle, sejoint6angle);
    legend('current angle', 'expected angle')
    grid on;
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint7angle));
    createjointangleplot(titlestr, stime,sjoint7angle, sejoint7angle);
    legend('current angle', 'expected angle')
    grid on;
    figure;
%     subplot(7,1,1);
    titlestr = sprintf('%s course',vname(sjoint1angle));
    createjointangleplot(titlestr, stime,sjoint1angle, seval_mat(:,8));
    legend('current angle', 'expected angle')
    grid on;
    
    
%     subplot(7,1,2);
%     titlestr = sprintf('%s course',vname(sjoint2angle));
% 	createjointangleplot(titlestr, stime,sjoint2angle, seval_mat(:,3));
%     subplot(7,1,3);
%     titlestr = sprintf('%s course',vname(sjoint3angle));
%     createjointangleplot(titlestr, stime,sjoint3angle, seval_mat(:,4));
%     subplot(7,1,4);
%     titlestr = sprintf('%s course',vname(sjoint4angle));
%     createjointangleplot(titlestr, stime,sjoint4angle, seval_mat(:,5));
%     subplot(7,1,5);
%     titlestr = sprintf('%s course',vname(sjoint5angle));
%     createjointangleplot(titlestr, stime,sjoint5angle, seval_mat(:,6));
%     subplot(7,1,6);
%     titlestr = sprintf('%s course',vname(sjoint6angle));
%     createjointangleplot(titlestr, stime,sjoint6angle, seval_mat(:,7));
%     subplot(7,1,7);
%     titlestr = sprintf('%s course',vname(sjoint7angle));
%     createjointangleplot(titlestr, stime,sjoint7angle, seval_mat(:,8));

    
    
    figure;
    hold all;
    title('jointangle course');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint1angle);
    plot(stime,sjoint2angle);
    plot(stime,sjoint3angle);
    plot(stime,sjoint4angle);
    plot(stime,sjoint5angle);
    plot(stime,sjoint6angle);
    plot(stime,sjoint7angle);
    legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
    hold off;
    
    figure;
    hold all;
    title('jointangle course');
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,sjoint1angle);
    plot(stime,sjoint2angle);
    plot(stime,sjoint3angle);
    plot(stime,sjoint4angle);
    plot(stime,sjoint5angle);
    plot(stime,sjoint6angle);
    plot(stime,sjoint7angle);
    legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
    hold off;
    
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

 
    
end

function createjointangleplot(titlestr, stime,jointangle, expangle)
    hold all;
    title(titlestr);
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,(jointangle));
    plot(stime,(expangle));
    %axis([0 20 -40 40])
    hold off;
end

function [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6] = readjointanglefile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f', 'headerLines', Headerlines)
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

function [Time,Angle_1,Angle_2,Angle_3,Angle_4,Angle_5,Angle_6,Angle_7] = readexpectedjointangle(fname)

    inputfile = fopen(fname);
    values = textscan(inputfile,'%f,%f,%f,%f,%f,%f,%f,%f');
    fclose(inputfile);
    Time = values{1};
    Angle_1= values{2};
    Angle_2= values{3};
    Angle_3= values{4};
    Angle_4= values{5};
    Angle_5= values{6};
    Angle_6= values{7};
    Angle_7= values{8};
end


function val = invscale(minangle,maxangle, propval)
    % see manager.py actuate_universal()
    LOWERBOUND = -1
    UPPERBOUND = +1
    
    %assert(minangle <= propval && maxangle >= propval);
    
    scaleorig = abs(UPPERBOUND - LOWERBOUND);
    scalemiddle = (UPPERBOUND + LOWERBOUND)/2
    scaledmiddledist = scalemiddle + propval;
    
    %assert(minangle < maxangle);
    
    scaletarget = abs(maxangle - minangle);
    
    fact =  scaletarget/ scaleorig;
      
    val = fact * scaledmiddledist;
end
