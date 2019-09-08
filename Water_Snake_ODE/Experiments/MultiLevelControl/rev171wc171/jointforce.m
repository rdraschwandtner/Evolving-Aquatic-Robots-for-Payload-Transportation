clear all;
%close all;

importfile('joint_feedback.out')

% import data with header
% timestamp,j1f1x,j1f1y,j1f1z,j1t1x,j1t1y,j1t1z,j1f2x,j1f2y,j1f2z,j1t2x,j1t2y,j1t2z,j2f1x,j2f1y,j2f1z,j2t1x,j2t1y,j2t1z,j2f2x,j2f2y,j2f2z,j2t2x,j2t2y,j2t2z,j3f1x,j3f1y,j3f1z,j3t1x,j3t1y,j3t1z,j3f2x,j3f2y,j3f2z,j3t2x,j3t2y,j3t2z,j4f1x,j4f1y,j4f1z,j4t1x,j4t1y,j4t1z,j4f2x,j4f2y,j4f2z,j4t2x,j4t2y,j4t2z,j5f1x,j5f1y,j5f1z,j5t1x,j5t1y,j5t1z,j5f2x,j5f2y,j5f2z,j5t2x,j5t2y,j5t2z,j6f1x,j6f1y,j6f1z,j6t1x,j6t1y,j6t1z,j6f2x,j6f2y,j6f2z,j6t2x,j6t2y,j6t2z,j7f1x,j7f1y,j7f1z,j7t1x,j7t1y,j7t1z,j7f2x,j7f2y,j7f2z,j7t2x,j7t2y,j7t2z

f1magnitude = [];
f2magnitude = [];
for row = 1:length(timestamp)
    rowf1magnitude = [];
    rowf2magnitude = [];
    for jointidx = 1:7
        varName1x = eval(['j' int2str(jointidx) 'f1x(row)']);
        varName1y = eval(['j' int2str(jointidx) 'f1y(row)']);
        varName1z = eval(['j' int2str(jointidx) 'f1z(row)']);

        rowf1magnitude = [rowf1magnitude,norm([varName1x,varName1y,varName1z])];

        varName2x = eval(['j' int2str(jointidx) 'f2x(row)']);
        varName2y = eval(['j' int2str(jointidx) 'f2y(row)']);
        varName2z = eval(['j' int2str(jointidx) 'f2z(row)']);

        rowf2magnitude = [rowf2magnitude,norm([varName2x,varName2y,varName2z])];
    end
    f1magnitude = [f1magnitude;rowf1magnitude];
    f2magnitude = [f2magnitude;rowf2magnitude];
end
figure(1);
hold all;
grid on;
title('force progress body 1');
ylabel('force (Nm)');
xlabel('time (sec)');
plot(timestamp, f1magnitude(:,1));
plot(timestamp, f1magnitude(:,2));
plot(timestamp, f1magnitude(:,3));
plot(timestamp, f1magnitude(:,4));
plot(timestamp, f1magnitude(:,5));
plot(timestamp, f1magnitude(:,6));
plot(timestamp, f1magnitude(:,7));
legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
hold off;

figure(2);
hold all;
grid on;
title('force progress body 2');
ylabel('force (Nm)');
xlabel('time (sec)');
plot(timestamp, f2magnitude(:,1));
plot(timestamp, f2magnitude(:,2));
plot(timestamp, f2magnitude(:,3));
plot(timestamp, f2magnitude(:,4));
plot(timestamp, f2magnitude(:,5));
plot(timestamp, f2magnitude(:,6));
plot(timestamp, f2magnitude(:,7));
legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7');
hold off;
