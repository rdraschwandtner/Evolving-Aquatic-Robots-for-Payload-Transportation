function MA_grasping_analysis() 
    close all;
    set(0,'defaultfigurecolor',[1 1 1])

    numofjoints = 7;
    jointidcs = 0:numofjoints-1;
    
    all_angles = ones(numofjoints,1)* 2*pi/(numofjoints+1);
    
    bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);
    calc_pos_relative_angle = zeros(1,length(all_angles(:,1)));
    
    figure(1);
    x0=100;y0=50;width=550;height=550;
    set(gcf,'units','points','position',[x0,y0,width,height]);
    grid on;
    xlim([-14 2]);
    ylim([-8 8]);
    ax = gca;
    set(ax,'XTick',[-14:2:2],'YTick',[-8:2:8],'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    
    bodies = bodymovement(bodies, calc_pos_relative_angle);
    printworm(bodies, 1);
    
    export_fig('graspingalgo_t0.pdf', '-painters', '-pdf','-p5');
    %pause;
    
    clf(1);
    x0=100;y0=50;width=550;height=550;
    set(gcf,'units','points','position',[x0,y0,width,height]);
    grid on;
    xlim([-14 2]);
    ylim([-8 8]);
    ax = gca;
    set(ax,'XTick',[-14:2:2],'YTick',[-8:2:8],'Fontsize',20);
    xlabel('X','Fontsize',25);
    ylabel('Z','Fontsize',25);
    bodies = bodymovement(bodies, all_angles);
    printworm(bodies, 1);    
    for i = 1:numofjoints-2
        exportstr = sprintf('graspingalgo_t%d.pdf',i);
        export_fig(exportstr, '-painters', '-pdf','-p5');
        %pause;
        clf(1);
        figure(1);
        grid on;
        xlim([-14 2]);
        ylim([-8 8]);
        ax = gca;
        set(gcf,'units','points','position',[x0,y0,width,height]);
        set(ax,'XTick',[-14:2:2],'YTick',[-8:2:8],'Fontsize',20);
        xlabel('X','Fontsize',25);
        ylabel('Z','Fontsize',25);

        all_angles_delta=[];
        angledelta = all_angles(i);
        all_angles_delta(i) = -angledelta;
        anglepart = angledelta/(numofjoints+1-i)
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
    exportstr = sprintf('graspingalgo_t%d.pdf',i);
    export_fig(exportstr, '-painters', '-pdf','-p5');

end

function to_val = convert_range(from_min, from_max, to_min, to_max, from_val)
    assert(from_max ~= from_min);
    from_range = (from_max - from_min);
    to_range = (to_max - to_min);
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min;
end