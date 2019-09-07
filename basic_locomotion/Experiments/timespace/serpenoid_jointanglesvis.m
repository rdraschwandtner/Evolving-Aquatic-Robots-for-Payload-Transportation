function joint_timespace_analysis() 
    close all;

    f = 1;
    w = 2*pi*f;
    
    dt = 0.02;
    n = 4;
    timestep = dt/n;
    t = 0:timestep:3;
    
    numofjoints = 8;
    %numofjoints = 16;
    jointidcs = 0:numofjoints-1;
    
    a = 0.2;
    %phi = 2 * pi / numofjoints; %shall be optimized
    %phi = pi / numofjoints;
    %phi = 2 * pi / (numofjoints+1);
    phi = 0.45;
    %phi = pi / (numofjoints) / 4;
    
    %offset = 0.1;
    offset = 0;
    
    cc=hsv(12);
    figure;
    hold on;
    grid on;
    title('angles per joint and time');
    all_angles = [];
    legendplots = [];
    for jointidx = jointidcs
        angles = a * sin(w*t-jointidx*phi) + offset;
        legendplots(jointidx+1) = plot3(t,ones(1,length(t))*jointidx,angles,'color',cc(jointidx+1,:));
        plot3(0,jointidx,angles(1),'r.', 'Markersize', 20);
        legendstr{jointidx+1} = sprintf('a*sin(wt+%d*phi)',jointidx);
        all_angles = [all_angles;angles];
    end

    
    xlabel('time (s)');
    ylabel('jointidx');
    zlabel('angle (-1,+1)');
    legend(legendplots,legendstr);
    hold off;
    