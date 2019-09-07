function grapsing_analysis() 
    close all;
    
    dt = 0.02;
    n = 4;
    timestep = dt/n;
    t = 0:timestep:10;    

    numofjoints = 16;
    jointidcs = 0:numofjoints-1;
    
    all_angles = ones(numofjoints,1)* 2*pi/(numofjoints+1);
    
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    calc_pos_relative_angle = zeros(1,length(all_angles(:,1)));
    
    
    BOUND = 30;
    figure(1);
    grid on;
    xlim([-BOUND BOUND]);
    ylim([-BOUND BOUND]);
    ax = gca;
    set(ax,'XTick',[-BOUND:2:BOUND]);
    set(ax,'YTick',[-BOUND:2:BOUND]);
    
    bodies = bodymovement(bodies, calc_pos_relative_angle);
    printworm(bodies, 1);
    
    pause;
    
    bodies = bodymovement(bodies, all_angles);
    printworm(bodies, 1);
    for i = 1:numofjoints
        pause;
        clf(1);
        figure(1);
        grid on;
        xlim([-BOUND BOUND]);
        ylim([-BOUND BOUND]);
        ax = gca;
        set(ax,'XTick',[-BOUND:2:BOUND]);
        set(ax,'YTick',[-BOUND:2:BOUND]);

        all_angles_delta=[];
        angledelta = all_angles(i);
        all_angles_delta(i) = -angledelta;
        anglepart = angledelta/(numofjoints-i)
        for jointidx = 1+i:numofjoints
            all_angles_delta(jointidx) = anglepart;
        end
        for jointidx = 1:i-1
            all_angles_delta(jointidx) = 0;
        end
        %all_angles_delta
        bodies = bodymovement(bodies, all_angles_delta);
        printworm(bodies, 1);
        all_angles = all_angles + all_angles_delta';
    end

end

function to_val = convert_range(from_min, from_max, to_min, to_max, from_val)
    assert(from_max ~= from_min);
    from_range = (from_max - from_min);
    to_range = (to_max - to_min);
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min;
end