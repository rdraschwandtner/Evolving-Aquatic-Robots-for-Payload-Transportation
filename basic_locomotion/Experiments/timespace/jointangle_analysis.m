function jointangle_analysis()
    close all;
    
    total_time = 2.5; %sec
    numbodies = 4;
    numjoints = numbodies-1;
    
    f = 1;
    w = 2*pi*f;
    a = 1;
    t = 0:0.02:total_time-1;
    
    figure;
    hold all;
    grid on;
    title('joint angle progress with +offset');
    legendstr = {};
    for jointidx = 1:numjoints
        phi = 2*pi/numjoints*(jointidx-1);
        y = a * sin(w*t+phi);
        plot(t,y);
        legendstr{jointidx} = sprintf('joint %d',jointidx);
    end
    legend(legendstr);
    xlabel('time');
    ylabel('joint angle (-1,+1)');
    hold off;
    
    figure;
    hold all;
    grid on;
    title('joint angle progress with -offset');
    legendstr = {};
    for jointidx = 1:numjoints
        phi = 2*pi/numjoints*(jointidx-1);
        y = a * sin(w*t-phi);
        plot(t,y);
        legendstr{jointidx} = sprintf('joint %d',jointidx);
    end
    legend(legendstr);
    xlabel('time');
    ylabel('joint angle (-1,+1)');
    hold off;
    
    figure;
    hold all;
    grid on;
    title('joint angle progress with +offset reverse joint order');
    legendstr = {};
    for jointidx = 1:numjoints
        phi = 2*pi/numjoints*(numjoints+1-jointidx);
        y = a * sin(w*t+phi);
        plot(t,y);
        legendstr{jointidx} = sprintf('joint %d',jointidx);
    end
    legend(legendstr);
    xlabel('time');
    ylabel('joint angle (-1,+1)');
    hold off;
    
    
end
