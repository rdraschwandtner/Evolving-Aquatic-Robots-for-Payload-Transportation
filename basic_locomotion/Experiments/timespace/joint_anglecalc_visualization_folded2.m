function joint_timespace_analysis2() 
    close all;

    % evolve forward swimming rep17, gen 999
    %f = 2.0017813097390436;
    
    % evolve folded forward swimming rep17, gen 999
    f = 1.717923228293962;
    
    w = 2*pi*f;
    
    dt = 0.02;
    n = 4;
    timestep = dt/n;
    t = 0:timestep:4;
    
    %numofjoints = 8;
    numofjoints = 16;
    jointidcs = 0:numofjoints-1;
    
    % evolve forward swimming rep17, gen 999
    %jointidcs_act = jointidcs;
    
    % evolve folded forward swimming rep17, gen 999
    jointidcs_fix = 0:7;
    jointidcs_act = 8:numofjoints-1;
    
    % evolve forward swimming rep17, gen 999
    %a = 0.458787405646039;
    
    % evolve folded forward swimming rep17, gen 999
    a = 0.7186821840063525;
    
    %phi = 2 * pi / numofjoints; %shall be optimized
    %phi = pi / numofjoints;
    %phi = 2 * pi / (numofjoints+1);
    %phi = pi / (numofjoints) / 4;
    
    % evolve forward swimming rep17, gen 999
    %phi = 0.6745837021712764;
    
    % evolve folded forward swimming rep17, gen 999
    phi = 5.363137316261791;
    
    %offset = 0.1;
    offset = 0;
    
    figure;
    hold all;
    grid on;
    title('angles per point');
    all_angles = [];
    
    % evolve folded forward swimming rep17, gen 999
    for jointidx = 0:2:4
        angle = 0;
        angles = ones(1,length(t)) * angle; % fixed over the whole time
        plot3(t,ones(1,length(t))*jointidx,angles);
        legendstr{jointidx+1} = sprintf('jointidx=%d',jointidx);
        all_angles = [all_angles;angles];
        angle = convert_range(-90, +90, -1, +1, -90);
        angles = ones(1,length(t)) * angle; % fixed over the whole time
        plot3(t,ones(1,length(t))*jointidx+1,angles);
        legendstr{jointidx+1+1} = sprintf('jointidx=%d',jointidx+1);
        all_angles = [all_angles;angles];
    end
    
    angle = 0;
    angles = ones(1,length(t)) * angle; % fixed over the whole time
    plot3(t,ones(1,length(t))*6,angles);
    legendstr{6+1} = sprintf('jointidx=%d',6);
    all_angles = [all_angles;angles];
    angle = convert_range(-90, +90, -1, +1, +45);
    angles = ones(1,length(t)) * angle; % fixed over the whole time
    plot3(t,ones(1,length(t))*7,angles);
    angles = ones(1,length(t)) * angle; % fixed over the whole time
    legendstr{7+1} = sprintf('jointidx=%d',7);
    all_angles = [all_angles;angles];
    
    
    for jointidx = jointidcs_act
        angles = a * sin(w*t-jointidx*phi) + offset;
        plot3(t,ones(1,length(t))*jointidx,angles);
        legendstr{jointidx+1} = sprintf('jointidx=%d',jointidx);
        all_angles = [all_angles;angles];
    end

% angles at timestep 0
%     all_angles = zeros(length(jointidcs),length(t));
%     
%     for i=1:2:length(t)
%         all_angles(5,i) = 1;
%     end
    
    
    
    xlabel('time (s)');
    ylabel('jointidx');
    zlabel('angle (-1,+1)');
    legend(legendstr);
    hold off;
    
    % plot angles
    
%     figure;
%     plot(jointidcs, all_angles(:,1),'+-');
    
    
%     figure;
%     hold on;
%     xlabel('relative x in resptect to the first segment');
%     ylabel('relative z in respect to the first segment');
%     segmentlength = 1.5;
%     
%     initsegment_front_pos_x = 0;
%     initsegment_back_pos_x = initsegment_front_pos_x - segmentlength;    
%     line([initsegment_front_pos_x initsegment_back_pos_x],[0 0],'Marker','.','LineStyle','-');
% 
%     for step = 1:length(t)
%         % timestep = 0
%         segment_front_pos_x = initsegment_back_pos_x;
%         segment_front_pos_y = 0;
%         old_pos_relative_angle = 0;
%         %legendstr = '';
%         for jointidx = jointidcs
%             calc_pos_relative_angle = all_angles(jointidx+1,step+1);
%             pos_relative_angle = old_pos_relative_angle + calc_pos_relative_angle;
%             draw_angle = convert_range(-1,+1, -pi/2,+pi/2, pos_relative_angle); % taken from simulation.py
%             draw_angle = pi - draw_angle;
%             segment_back_pos_x = segment_front_pos_x + cos(draw_angle)*segmentlength;
%             segment_back_pos_y = segment_front_pos_y + sin(draw_angle)*segmentlength;
%             line([segment_front_pos_x segment_back_pos_x],[segment_front_pos_y segment_back_pos_y],'Marker','.','LineStyle','-');
%             segment_front_pos_x = segment_back_pos_x;
%             segment_front_pos_y = segment_back_pos_y;
%             old_pos_relative_angle = pos_relative_angle;
%             %legendstr = strcat(legendstr,sprintf('a%d=%.2f,',jointidx,calc_pos_relative_angle));
%         end
%         title(sprintf('timestep= %.3f (s)',t(step)));
%         %legend(legendstr);
%         %pause(0.1);
%         pause(timestep);
%         
%     end
    
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    %calc_pos_relative_angle = zeros(1,length(all_angles(:,1)));
    calc_pos_relative_angle = zeros(1,length(all_angles(:,1)));
    %calc_pos_relative_angle(1) = -pi/2;
    
    BOUND = 30;
    figure(3);
    grid on;
    xlim([-BOUND BOUND]);
    ylim([-BOUND BOUND]);
    ax = gca;
    set(ax,'XTick',[-BOUND:2:BOUND]);
    set(ax,'YTick',[-BOUND:2:BOUND]);
    
    bodies = bodymovement(bodies, calc_pos_relative_angle);
    printworm(bodies, 3);
    lastangles = calc_pos_relative_angle';
    for step = 1:10:length(t)
        calc_pos_relative_angle = all_angles(:,step);
        draw_angles = convert_range(-1,+1, -pi/2,+pi/2, calc_pos_relative_angle);
        %draw_angles = pi - draw_angles;
        diff_angles = draw_angles - lastangles; % bodymovement uses delta angles!
        bodies = bodymovement(bodies, diff_angles);
        lastangles = draw_angles;
        printworm(bodies, 3);
        pause;
        clf(3);
        figure(3);
        grid on;
        xlim([-BOUND BOUND]);
        ylim([-BOUND BOUND]);
        ax = gca;
        set(ax,'XTick',[-BOUND:2:BOUND]);
        set(ax,'YTick',[-BOUND:2:BOUND]);
    end
    
    
    
    
 
end

function to_val = convert_range(from_min, from_max, to_min, to_max, from_val)
    assert(from_max ~= from_min);
    from_range = (from_max - from_min);
    to_range = (to_max - to_min);
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min;
end