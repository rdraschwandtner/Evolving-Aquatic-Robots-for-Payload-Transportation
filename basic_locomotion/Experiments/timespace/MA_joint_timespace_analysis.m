function joint_timespace_analysis()

    % pdf readers usually render the 3d plot wrong!
    % Solution
    % http://www.mathworks.com/matlabcentral/newsreader/view_thread/280185
    % find a way to turn the anti-aliasing (in your PDF/EPS viewer) off,
    % I think the problem will likely go away. E.g. if you are using 
    % Adobe Reader: go to Preferences > Page Display > disable the 
    % "smooth line art" setting.

    close all;
    
    numofjoints = 7;
    k = 2*pi/numofjoints; % saito approximation
    f = 1;
    w = 2*pi*f;    
    
    x = 0:0.1:numofjoints-1;
    t = 0:0.1/2:(numofjoints-1)/2;

    [Y,X] = meshgrid(x,t);
    Z = sin(w*X-k*Y);
    
    set(0,'defaultfigurecolor',[1 1 1]);
    figure;
    %legendstr = {};
    set(gca,'YTick',0:1:7,'Fontsize',20);
    hold all;
    grid on;
    %title('joint angle progress per joint index');
    %surf(X,Y,Z, 'FaceAlpha',0.4); % not possible to export
    surf(X,Y,Z);
    %colormap(gray);
    shading interp;
    xlabel('time (s)','Fontsize',20);
    ylabel('joint index','Fontsize',20);
    zlabel('joint angle','Fontsize',20);
    
    func = @(x) colorspace('RGB->Lab',x);
    colors = distinguishable_colors(8,'w',func);
    
    plot3(t,zeros(1,length(x)),sin(t*w), 'color','black','LineWidth',4);
    %legendstr{2} = 'sin(2\pi*t - 0*2\pi/8)';
    plot3(zeros(1,length(x)),x,sin(-x*k), '-.','color','black','LineWidth',3);
    %legendstr{1} = 'body wave';
    %plot3(t,x,sin(w*t), 'black','LineWidth',3);
    %plot3(t,x,sin(w*t), 'g*','LineWidth',1);
    %colors = jet(6);

    for jointidx = 1:numofjoints-2
        angles = 1 * sin(w*t-jointidx*k);
        plot3(t,ones(1,length(t))*jointidx,angles,'LineWidth',4,'color',colors(jointidx,:));
        plot3(t,ones(1,length(t))*(jointidx),angles,'LineWidth',2,'color','black');
        %plot3(t,ones(1,length(t))*(jointidx+0.02),angles,'LineWidth',2,'color','black');
        %legendstr{2+jointidx} = sprintf('sin(2*pi*t - %d*2*pi/8)',jointidx);
    end
    angles = 1 * sin(w*t-(jointidx+1)*k);
    plot3(t,ones(1,length(t))*(jointidx+1),angles,'color',colors(8,:),'LineWidth',4); % set the last explicitly to green
    plot3(t,ones(1,length(t))*(jointidx+1),angles,'LineWidth',2,'color','black');
    %legendstr{7} = 'sin(2*pi*t - 7*2*pi/8)';
    %legend(p,legendstr);
    axis([0 (numofjoints-1)/2 0 numofjoints-1 -1 1]);
    hold off;
    
    % use export figure by hand
    %export_fig('joint_angle_progress_per_joint_index.pdf', '-pdf','-painters','-p5');
    
    pause;
    t = 0:0.1/10:(numofjoints-1)/2;
    figure;    
    %set(gca,'Fontsize',10);
    %title('joint angle progress for joint indices 0 and 1');
    hold on;
    ylim([-1.1 1.1]);
    ax = gca;
    set(ax,'YTick',[-1.0:0.2:1.0],'Fontsize',10);
    %grid on;
    angle = 1 * sin(w*t-1*k);
    h2 = plot(t,angle,'color',colors(1,:),'LineWidth',2);
    plot(t,angle,'color','black','LineWidth',1);
    h1 = plot(t,sin(t*w), 'color','black','LineWidth',2); 
    
    %axis([0 7/2 -1 1]);
    legend([h1 h2],'jointidx 0', 'jointidx 1');
    xlabel('time (s)','Fontsize',15);
    ylabel('joint angle','Fontsize',15);
    hold off;
    % adjust legend and execute this command
    %export_fig('joint_angle_progress_for_joint_indices_0_and_1.pdf', '-pdf','-painters','-p5');
     
    
    
    
end