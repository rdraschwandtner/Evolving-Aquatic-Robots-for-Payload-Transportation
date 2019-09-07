function joint_timespace_analysis() 
    close all;
    
    set(0,'defaultfigurecolor',[1 1 1])
  
    numofjoints = 16;
    jointidcs = 0:numofjoints-1;
    
    limx = [-26 0];
    limy = [-13 13];
    xtick = [-26:2:0];
    ytick = [-13:2:13];
    
    
    
    %% forward locomotion

    f = 1;
    w = 2*pi*f;

    t = 0:1/4:1-1/4;
    phi = 2*pi / numofjoints;
    offset = 0;
    a = 0.1;
    all_angles = [];
    for jointidx = jointidcs
        angles = a * sin(w*t-jointidx*phi) + offset;
        all_angles = [all_angles;angles];
    end

    bodies = create_worm(numofjoints+1,1.5,[0,0]);
    
    figure(1);
    
    lastangles = zeros(numofjoints,1);
    for tstep = 1:1:length(t)
        %pause;
        clf(1);
        figure(1);
        grid on;
        xlim(limx);
        ylim(limy);
        ax = gca;
        set(ax,'XTick',xtick,'YTick',ytick,'Fontname','Helvetica','Fontsize',12);
        xlabel('X','Fontname','Helvetica','Fontsize',15);
        ylabel('Z','Fontname','Helvetica','Fontsize',15);
        
        calc_pos_relative_angle = all_angles(:,tstep);
        draw_angles = convert_range(-1,+1, -pi/2,+pi/2, calc_pos_relative_angle);

        diff_angles = draw_angles - lastangles; % bodymovement uses delta angles!
        bodies = bodymovement(bodies, diff_angles);
        lastangles = draw_angles;
        printworm(bodies, 1);
        expfname = sprintf('forwardlocomotion_tstep%d.pdf',tstep);
        export_fig(expfname, '-painters', '-pdf','-p5');
    end
    
    display('forward locomotion finished');
    pause;    
   
    
    %% circular motion
    
    offset = 0.05;

    all_angles = [];
    for jointidx = jointidcs
        angles = a * sin(w*t-jointidx*phi) + offset;
        all_angles = [all_angles;angles];
    end

    bodies = create_worm(numofjoints+1,1.5,[0,0]);
    
    figure(1);
    
    lastangles = zeros(numofjoints,1);
    for tstep = 1:1:length(t)
        %pause;
        clf(1);
        figure(1);
        grid on;
        xlim(limx);
        ylim(limy);
        ax = gca;
        set(ax,'XTick',xtick,'YTick',ytick,'Fontname','Helvetica','Fontsize',12);
        xlabel('X','Fontname','Helvetica','Fontsize',15);
        ylabel('Z','Fontname','Helvetica','Fontsize',15);
        
        calc_pos_relative_angle = all_angles(:,tstep);
        draw_angles = convert_range(-1,+1, -pi/2,+pi/2, calc_pos_relative_angle);

        diff_angles = draw_angles - lastangles; % bodymovement uses delta angles!
        bodies = bodymovement(bodies, diff_angles);
        lastangles = draw_angles;
        printworm(bodies, 1);
        expfname = sprintf('circularmotion_tstep%d.pdf',tstep);
        export_fig(expfname, '-painters', '-pdf','-p5');
    end
    
    display('circular motion finished');
    pause;
    
    %% spinning
    offset = 0;

    all_angles = [];
    for jointidx = jointidcs
        if jointidx < numofjoints/2 % e.g. 4
            beta = jointidx-numofjoints/2;
            offset = +0.3;
        else
            beta = numofjoints/2-jointidx;
            offset = -0.3;
        end
        angles = a * sin(w*t-beta*phi) + offset;
        all_angles = [all_angles;angles];
    end

    bodies = create_worm(numofjoints+1,1.5,[0,0]);
    
    figure(1);
    
    lastangles = zeros(numofjoints,1);
    for tstep = 1:1:length(t)
        %pause;
        clf(1);
        figure(1);
        grid on;
        xlim(limx);
        ylim(limy);
        ax = gca;
        set(ax,'XTick',xtick,'YTick',ytick,'Fontname','Helvetica','Fontsize',12);
        xlabel('X','Fontname','Helvetica','Fontsize',15);
        ylabel('Z','Fontname','Helvetica','Fontsize',15);
        
        calc_pos_relative_angle = all_angles(:,tstep);
        draw_angles = convert_range(-1,+1, -pi/2,+pi/2, calc_pos_relative_angle);

        diff_angles = draw_angles - lastangles; % bodymovement uses delta angles!
        bodies = bodymovement(bodies, diff_angles);
        lastangles = draw_angles;
        printworm(bodies, 1);
        expfname = sprintf('spinning_motion_tstep%d.pdf',tstep);
        export_fig(expfname, '-painters', '-pdf','-p5');
    end
    
    display('spinning motion finished');
    pause;

    %% coiling
    offset = 0;
    all_angles = [];
    for jointidx = jointidcs
        angles = a * sin(w*t-jointidx*phi) + offset;
        all_angles = [all_angles;angles];
    end
    
    for jointidx = 1:numofjoints
        idcs = find(all_angles(jointidx,:)>a/2);
        for idx=idcs
            [temp,len] = size(all_angles);
            if idx < len
                %all_angles(jointidx,:) = [all_angles(jointidx,1:idx) all_angles(jointidx,idx) all_angles(jointidx,idx+1:end-1)];
                all_angles(jointidx,idx:end) = all_angles(jointidx,idx);
            end
        end
    end

    bodies = create_worm(numofjoints+1,1.5,[0,0]);
    
    figure(1);
    
    lastangles = zeros(numofjoints,1);
    for tstep = 1:1:length(t)
        %pause;
        clf(1);
        figure(1);
        grid on;
        xlim(limx);
        ylim(limy);
        ax = gca;
        set(ax,'XTick',xtick,'YTick',ytick,'Fontname','Helvetica','Fontsize',12);
        xlabel('X','Fontname','Helvetica','Fontsize',15);
        ylabel('Z','Fontname','Helvetica','Fontsize',15);
        
        calc_pos_relative_angle = all_angles(:,tstep);
        draw_angles = convert_range(-1,+1, -pi/2,+pi/2, calc_pos_relative_angle);

        diff_angles = draw_angles - lastangles; % bodymovement uses delta angles!
        bodies = bodymovement(bodies, diff_angles);
        lastangles = draw_angles;
        printworm(bodies, 1);
        expfname = sprintf('coil_tstep%d.pdf',tstep);
        export_fig(expfname, '-painters', '-pdf','-p5');
    end
    
    display('coiling finished');
    pause;  
end

function to_val = convert_range(from_min, from_max, to_min, to_max, from_val)
    assert(from_max ~= from_min);
    from_range = (from_max - from_min);
    to_range = (to_max - to_min);
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min;
end