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
    sjoint1angle = sval_mat(:,3);
    sjoint2angle = sval_mat(:,5);
    sjoint3angle = sval_mat(:,7);
    sjoint4angle = sval_mat(:,9);
    sjoint5angle = sval_mat(:,11);
    sjoint6angle = sval_mat(:,13);
    sjoint7angle = sval_mat(:,15);
    
    [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7] = readexpectedjointangle('expected_joint_angles.out');
    eval_mat = [eTime,eAngle_1,eAngle_2,eAngle_3,eAngle_4,eAngle_5,eAngle_6,eAngle_7];
    seval_mat = sortrows(eval_mat, 1);
    setime = seval_mat(:,1);    
    sejoint1angle = seval_mat(:,2);
    sejoint2angle = seval_mat(:,3);
    sejoint3angle = seval_mat(:,4);
    sejoint4angle = seval_mat(:,5);
    sejoint5angle = seval_mat(:,6);
    sejoint6angle = seval_mat(:,7);
    sejoint7angle = seval_mat(:,8); 
    
    figure;
    hold all;
    title('jointidx 0 angle course');
    ylabel('joint angle (-1,..,+1)');
    xlabel('time');
    plot(stime,sjoint1angle);
    plot(stime,sejoint1angle);
    legend ('actual joint angle','expected joint angle');
    grid on;
%     plot(stime,sjoint2angle);
%     plot(stime,sjoint3angle);
%     plot(stime,sjoint4angle);
%     plot(stime,sjoint5angle);
%     plot(stime,sjoint6angle);
%     plot(stime,sjoint7angle);
%     legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
    hold off;

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
    values = textscan(inputfile,'%f,|[%f, %f, %f, %f, %f, %f, %f]|');
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
