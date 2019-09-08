function jointanalysis21()
    close all;
    jointfilestr = sprintf('validator_logging\\joint_angles\\rep0_joint_angles.dat');    
    [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6,Angle_1_7,Angle_2_7,Angle_1_8,...
    Angle_2_8,Angle_1_9,Angle_2_9,Angle_1_10,Angle_2_10,Angle_1_11,Angle_2_11,Angle_1_12,Angle_2_12,...
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15,Angle_1_16,Angle_2_16,Angle_1_17,...
    Angle_2_17,Angle_1_18,Angle_2_18,Angle_1_19,Angle_2_19,Angle_1_20,Angle_2_20] = ...
    readjointanglefile(jointfilestr);

    val_mat = [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6,Angle_1_7,Angle_2_7,Angle_1_8,...
    Angle_2_8,Angle_1_9,Angle_2_9,Angle_1_10,Angle_2_10,Angle_1_11,Angle_2_11,Angle_1_12,Angle_2_12,...
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15,Angle_1_16,Angle_2_16,Angle_1_17,...
    Angle_2_17,Angle_1_18,Angle_2_18,Angle_1_19,Angle_2_19,Angle_1_20,Angle_2_20];

    sval_mat = sortrows(val_mat, 1)
    stime = sval_mat(:,1);
    sjoint1angle = invscale(-pi,pi,sval_mat(:,3));
    sjoint2angle = invscale(-pi,pi,sval_mat(:,5));
    sjoint3angle = invscale(-pi,pi,sval_mat(:,7));
    sjoint4angle = invscale(-pi,pi,sval_mat(:,9));
    sjoint5angle = invscale(-pi,pi,sval_mat(:,11));
    sjoint6angle = invscale(-pi,pi,sval_mat(:,13));
    sjoint7angle = invscale(-pi,pi,sval_mat(:,15));
    sjoint8angle = invscale(-pi,pi,sval_mat(:,17));
    sjoint9angle = invscale(-pi,pi,sval_mat(:,19));
    sjoint10angle = invscale(-pi,pi,sval_mat(:,21));
    sjoint11angle = invscale(-pi,pi,sval_mat(:,23));
    sjoint12angle = invscale(-pi,pi,sval_mat(:,25));
    sjoint13angle = invscale(-pi,pi,sval_mat(:,27));
    sjoint14angle = invscale(-pi,pi,sval_mat(:,29));
    sjoint15angle = invscale(-pi,pi,sval_mat(:,31));
    sjoint16angle = invscale(-pi,pi,sval_mat(:,33));
    sjoint17angle = invscale(-pi,pi,sval_mat(:,35));
    sjoint18angle = invscale(-pi,pi,sval_mat(:,37));
    sjoint19angle = invscale(-pi,pi,sval_mat(:,39));
    sjoint20angle = invscale(-pi,pi,sval_mat(:,41));
    sjoint21angle = invscale(-pi,pi,sval_mat(:,43));
    
    
    
    
    [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7,eAngle_8,...
    eAngle_9,eAngle_10,eAngle_11,eAngle_12,eAngle_13,eAngle_14,eAngle_15,eAngle_16,eAngle_17,...
    eAngle_18,eAngle_19,eAngle_20,Angle_21] = readexpectedjointangle('expected_joint_angles.out')

    eval_mat = [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7,eAngle_8,...
    eAngle_9,eAngle_10,eAngle_11,eAngle_12,eAngle_13,eAngle_14,eAngle_15,eAngle_16,eAngle_17,...
    eAngle_18,eAngle_19,eAngle_20,Angle_21];

    seval_mat = sortrows(eval_mat, 1)
    setime = seval_mat(:,1);
    %assert(isequal(stime,setime)) % attention: these are floating point values!
    
    
    
    vname=@(x) inputname(1); %http://www.mathworks.com/matlabcentral/newsreader/view_thread/251347
    
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint1angle));
    createjointangleplot(titlestr, stime,sjoint1angle, invscale(-pi,pi,seval_mat(:,2)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint2angle));
    createjointangleplot(titlestr, stime,sjoint2angle, invscale(-pi,pi,seval_mat(:,3)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint3angle));
    createjointangleplot(titlestr, stime,sjoint3angle, invscale(-pi,pi,seval_mat(:,4)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint4angle));
    createjointangleplot(titlestr, stime,sjoint4angle, invscale(-pi,pi,seval_mat(:,5)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint5angle));
    createjointangleplot(titlestr, stime,sjoint5angle, invscale(-pi,pi,seval_mat(:,6)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint6angle));
    createjointangleplot(titlestr, stime,sjoint6angle, invscale(-pi,pi,seval_mat(:,7)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint7angle));
    createjointangleplot(titlestr, stime,sjoint7angle, invscale(-pi,pi,seval_mat(:,8)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint8angle));
    createjointangleplot(titlestr, stime,sjoint8angle, invscale(-pi,pi,seval_mat(:,9)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint9angle));
    createjointangleplot(titlestr, stime,sjoint9angle, invscale(-pi,pi,seval_mat(:,10)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint10angle));
    createjointangleplot(titlestr, stime,sjoint10angle, invscale(-pi,pi,seval_mat(:,11)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint11angle));
    createjointangleplot(titlestr, stime,sjoint11angle, invscale(-pi,pi,seval_mat(:,12)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint12angle));
    createjointangleplot(titlestr, stime,sjoint12angle, invscale(-pi,pi,seval_mat(:,13)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint13angle));
    createjointangleplot(titlestr, stime,sjoint13angle, invscale(-pi,pi,seval_mat(:,14)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint14angle));
    createjointangleplot(titlestr, stime,sjoint14angle, invscale(-pi,pi,seval_mat(:,15)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint15angle));
    createjointangleplot(titlestr, stime,sjoint15angle, invscale(-pi,pi,seval_mat(:,16)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint16angle));
    createjointangleplot(titlestr, stime,sjoint16angle, invscale(-pi,pi,seval_mat(:,17)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint17angle));
    createjointangleplot(titlestr, stime,sjoint17angle, invscale(-pi,pi,seval_mat(:,18)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint18angle));
    createjointangleplot(titlestr, stime,sjoint18angle, invscale(-pi,pi,seval_mat(:,19)));
    legend('current angle', 'expected angle')
    grid on;
    
    figure;
    subplot(3,1,1);
    titlestr = sprintf('%s course',vname(sjoint19angle));
    createjointangleplot(titlestr, stime,sjoint19angle, invscale(-pi,pi,seval_mat(:,20)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,2);
    titlestr = sprintf('%s course',vname(sjoint20angle));
    createjointangleplot(titlestr, stime,sjoint20angle, invscale(-pi,pi,seval_mat(:,21)));
    legend('current angle', 'expected angle')
    grid on;
    subplot(3,1,3);
    titlestr = sprintf('%s course',vname(sjoint21angle));
    createjointangleplot(titlestr, stime,sjoint21angle, invscale(-pi,pi,seval_mat(:,22)));
    legend('current angle', 'expected angle')
    grid on;
    
    
%     figure;
%     hold all;
%     title('jointangle course');
%     ylabel('joint angle (rad)');
%     xlabel('time');
%     plot(stime,sjoint1angle);
%     plot(stime,sjoint2angle);
%     plot(stime,sjoint3angle);
%     plot(stime,sjoint4angle);
%     plot(stime,sjoint5angle);
%     plot(stime,sjoint6angle);
%     plot(stime,sjoint7angle);
%     legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
%     hold off;
    

 
    
end

function createjointangleplot(titlestr, stime,jointangle, expangle)
    hold all;
    title(titlestr);
    ylabel('joint angle (rad)');
    xlabel('time');
    plot(stime,radtodeg(jointangle));
    plot(stime,radtodeg(expangle));
    hold off;
end

function [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6,Angle_1_7,Angle_2_7,Angle_1_8,...
    Angle_2_8,Angle_1_9,Angle_2_9,Angle_1_10,Angle_2_10,Angle_1_11,Angle_2_11,Angle_1_12,Angle_2_12,...
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15,Angle_1_16,Angle_2_16,Angle_1_17,...
    Angle_2_17,Angle_1_18,Angle_2_18,Angle_1_19,Angle_2_19,Angle_1_20,Angle_2_20] = readjointanglefile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f', 'headerLines', Headerlines)
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
    Angle_1_7 = values{16};
    Angle_2_7 = values{17};
    Angle_1_8 = values{18};
    Angle_2_8 = values{19};
    Angle_1_9 = values{20};
    Angle_2_9 = values{21};
    Angle_1_10 = values{22};
    Angle_2_10 = values{23};
    Angle_1_11 = values{24};
    Angle_2_11 = values{25};
    Angle_1_12 = values{26};
    Angle_2_12 = values{27};
    Angle_1_13 = values{28};
    Angle_2_13 = values{29};
    Angle_1_14 = values{30};
    Angle_2_14 = values{31};
    Angle_1_15 = values{32};
    Angle_2_15 = values{33};
    Angle_1_16 = values{34};
    Angle_2_16 = values{35};
    Angle_1_17 = values{36};
    Angle_2_17 = values{37};
    Angle_1_18 = values{38};
    Angle_2_18 = values{39};
    Angle_1_19 = values{40};
    Angle_2_19 = values{41};
    Angle_1_20 = values{42};
    Angle_2_20 = values{43};
end

function [Time,Angle_1,Angle_2,Angle_3,Angle_4,Angle_5,Angle_6,Angle_7,Angle_8,...
    Angle_9,Angle_10,Angle_11,Angle_12,Angle_13,Angle_14,Angle_15,Angle_16,Angle_17,...
    Angle_18,Angle_19,Angle_20,Angle_21] = readexpectedjointangle(fname)

    inputfile = fopen(fname);
    values = textscan(inputfile,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    fclose(inputfile);
    Time = values{1};
    Angle_1= values{2};
    Angle_2= values{3};
    Angle_3= values{4};
    Angle_4= values{5};
    Angle_5= values{6};
    Angle_6= values{7};
    Angle_7= values{8};
    Angle_8= values{9};
    Angle_9= values{10};
    Angle_10= values{11};
    Angle_11= values{12};
    Angle_12= values{13};
    Angle_13= values{14};
    Angle_14= values{15};
    Angle_15= values{16};
    Angle_16= values{17};
    Angle_17= values{18};
    Angle_18= values{19};
    Angle_19= values{20};
    Angle_20= values{21};
    Angle_21= values{22};
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
