function jointanalysis21()
    close all;
    jointfilestr = sprintf('validator_logging\\joint_angles\\_joint_angles.dat');    
    [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6,Angle_1_7,Angle_2_7,Angle_1_8,...
    Angle_2_8,Angle_1_9,Angle_2_9,Angle_1_10,Angle_2_10,Angle_1_11,Angle_2_11,Angle_1_12,Angle_2_12,...
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15] = ...
    readjointanglefile(jointfilestr);

    val_mat = [Time,Angle_1_0,Angle_2_0,Angle_1_1,Angle_2_1,Angle_1_2,Angle_2_2,Angle_1_3,Angle_2_3,...
    Angle_1_4,Angle_2_4,Angle_1_5,Angle_2_5,Angle_1_6,Angle_2_6,Angle_1_7,Angle_2_7,Angle_1_8,...
    Angle_2_8,Angle_1_9,Angle_2_9,Angle_1_10,Angle_2_10,Angle_1_11,Angle_2_11,Angle_1_12,Angle_2_12,...
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15];

    sval_mat = sortrows(val_mat, 1)
    stime = sval_mat(:,1);
    sjoint1angle = sval_mat(:,3);
    sjoint2angle = sval_mat(:,5);
    sjoint3angle = sval_mat(:,7);
    sjoint4angle = sval_mat(:,9);
    sjoint5angle = sval_mat(:,11);
    sjoint6angle = sval_mat(:,13);
    sjoint7angle = sval_mat(:,15);
    sjoint8angle = sval_mat(:,17);
    sjoint9angle = sval_mat(:,19);
    sjoint16angle = sval_mat(:,33);
    
    
    
    
    [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7,eAngle_8,...
    eAngle_9,eAngle_10,eAngle_11,eAngle_12,eAngle_13,eAngle_14,eAngle_15,eAngle_16...
    ] = readexpectedjointangle('expected_joint_angles.out')

    eval_mat = [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7,eAngle_8,...
    eAngle_9,eAngle_10,eAngle_11,eAngle_12,eAngle_13,eAngle_14,eAngle_15,eAngle_16];

    seval_mat = sortrows(eval_mat, 1);
    setime = seval_mat(:,1);
    %assert(isequal(stime,setime)) % attention: these are floating point values!
    sejoint1angle = seval_mat(:,2);
    sejoint8angle = seval_mat(:,9);
    sejoint9angle = seval_mat(:,10);
    sejoint16angle = seval_mat(:,17);
    
    
    vname=@(x) inputname(1); %http://www.mathworks.com/matlabcentral/newsreader/view_thread/251347
    
    
    figure;
    hold on;
    title('joint value course');
    plot(stime,sjoint1angle,'b*');
    plot(stime,sejoint1angle,'b');
    plot(stime,sjoint16angle,'g*');
    plot(stime,sejoint16angle,'g');
    %legend('jointidx0','jointidx15')
    legend('act jointidx0','calc jointidx0','act jointidx15','calc jointidx15')
    grid on;
    
    figure;
    hold on;
    title('joint value course');
    %plot(stime,sjoint8angle,'b');
    plot(stime,sejoint8angle,'b');
    %plot(stime,sjoint9angle,'g');
    plot(stime,sejoint9angle,'g');
    legend('jointidx7','jointidx8')
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
    Angle_1_13,Angle_2_13,Angle_1_14,Angle_2_14,Angle_1_15,Angle_2_15...
    ] = readjointanglefile(fname)
    inputfile = fopen(fname);
    Headerlines = 1;
    
    values = textscan(inputfile,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f', 'headerLines', Headerlines)
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
end

function [Time,Angle_1,Angle_2,Angle_3,Angle_4,Angle_5,Angle_6,Angle_7,Angle_8,...
    Angle_9,Angle_10,Angle_11,Angle_12,Angle_13,Angle_14,Angle_15,Angle_16...
    ] = readexpectedjointangle(fname)

    inputfile = fopen(fname);
    values = textscan(inputfile,'%f,|[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]|');
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

end

