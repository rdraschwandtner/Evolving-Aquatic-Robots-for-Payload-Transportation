function joint_timespace_analysis() 
    close all;

    f = 1;
    w = 2*pi*f;
    
    dt = 0.02;
    n = 4;
    timestep = dt/n;
    t = 0:timestep:10;
    
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
    
    figure;
    hold all;
    grid on;
    title('angles per point');
    all_angles = [];
    for jointidx = jointidcs
        angles = a * sin(w*t-jointidx*phi) + offset;
        plot3(t,ones(1,length(t))*jointidx,angles);
        legendstr{jointidx+1} = sprintf('jointidx=%d',jointidx);
        all_angles = [all_angles;angles];
    end
    
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
    
    figure;
    plot(jointidcs, all_angles(:,1),'+-');
    
    
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