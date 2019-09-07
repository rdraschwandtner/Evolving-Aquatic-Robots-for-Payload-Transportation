function joint_timespace_analysis()
    close all;

    k = 2*pi/8; % saito approximation
    f = 1;
    w = 2*pi*f;    
    
    x = 0:0.05:7;
    t = 0:0.05/2:7/2;   

    [Y,X] = meshgrid(x,t);
    Z = sin(w*X-k*Y);
    
    figure;
    legendstr = {};
    set(gca,'FontSize',18,'FontName','Garamond');
    hold all;
    grid on;
    title('joint angle progress per joint index');
    surf(X,Y,Z, 'FaceAlpha',0.4);
    shading interp;
    xlabel('time (s)');
    ylabel('joint index');
    zlabel('joint angle (-1,+1)');
    p = [];
    plot3(t,zeros(1,length(x)),sin(t*w), 'black','LineWidth',2);
    legendstr{2} = 'sin(2\pi*t - 0*2\pi/8)';
    plot3(zeros(1,length(x)),x,sin(-x*k), 'black--','LineWidth',2);
    legendstr{1} = 'body wave';
    %plot3(t,x,sin(w*t), 'black','LineWidth',3);
    %plot3(t,x,sin(w*t), 'g*','LineWidth',1);
    
    for jointidx = 1:6
        angles = 1 * sin(w*t-jointidx*k);
        plot3(t,ones(1,length(t))*jointidx,angles,'LineWidth',2);
        legendstr{2+jointidx} = sprintf('sin(2*pi*t - %d*2*pi/8)',jointidx);
    end
    angles = 1 * sin(w*t-(jointidx+1)*k);
    plot3(t,ones(1,length(t))*(jointidx+1),angles,'green','LineWidth',2); % set the last explicitly to green
    legendstr{7} = 'sin(2*pi*t - 7*2*pi/8)';
    %legend(p,legendstr);
    axis([0 7/2 0 7 -1 1]);
    hold off;
     
     
    
    
    
end