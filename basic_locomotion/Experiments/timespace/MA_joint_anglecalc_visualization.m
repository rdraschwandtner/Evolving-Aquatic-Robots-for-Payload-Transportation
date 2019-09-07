function joint_timespace_analysis() 
    close all;
    
    set(0,'defaultfigurecolor',[1 1 1])
  
    numofjoints = 7;
    jointidcs = 0:numofjoints-1;
    
    limx = [-14 2];
    limy = [-6 6];
    xtick = [-14:2:2];
    ytick = [-6:2:6];
    
    
    
    %% initial position
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    
    figure(1);
    grid on;
    xlim(limx);
    ylim(limy);
    ax = gca;
    set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    printworm(bodies, 1);
    export_fig('shapevisualizer_init.pdf', '-pdf','-p5');
    
    %% middle joint 90 degree
    all_angles = [zeros(numofjoints,1)];
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    calc_pos_relative_angle = zeros(1,length(all_angles(:,1)));
    calc_pos_relative_angle(3+1) = convert_range(-90, +90, -pi/2, +pi/2, 90);
    bodies = bodymovement(bodies, calc_pos_relative_angle);
    
    figure(2);
    grid on;
    xlim(limx);
    ylim(limy);
    ax = gca;
    set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    printworm(bodies, 2);
    export_fig('shapevisualizer_joint3_90deg.pdf', '-pdf','-p5');
    
    %% circle
    all_angles = [zeros(numofjoints,1)];
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    calc_pos_relative_angle = ones(1,length(all_angles(:,1))) * convert_range(-180, +180, -pi, +pi, 360/(numofjoints+1));
    bodies = bodymovement(bodies, calc_pos_relative_angle);
    
    figure(3);
    grid on;
    xlim(limx);
    ylim(limy);
    ax = gca;
    set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    printworm(bodies, 3);
    export_fig('shapevisualizer_circle.pdf', '-pdf','-p5');
    
    %% sine
    t = 0;
    w = 2*pi*1;
    a = 1;
    phi = 2*pi/numofjoints;
    all_angles=[];
    for jointidx = jointidcs
        all_angles = [all_angles;a * sin(w*t-jointidx*phi) + 0];
    end
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    bodies = bodymovement(bodies, all_angles);

    figure(4);
    grid on;
    xlim(limx);
    ylim(limy);
    ax = gca;
    set(ax,'XTick',xtick,'YTick',ytick,'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    printworm(bodies, 4);
    export_fig('shapevisualizer_sine.pdf', '-pdf','-p5');
end

function to_val = convert_range(from_min, from_max, to_min, to_max, from_val)
    assert(from_max ~= from_min);
    from_range = (from_max - from_min);
    to_range = (to_max - to_min);
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min;
end