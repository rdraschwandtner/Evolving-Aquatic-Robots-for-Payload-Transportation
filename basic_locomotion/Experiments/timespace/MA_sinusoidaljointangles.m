close all;
numofjoints = 7;
jointidcs = 0:numofjoints-1;
t = 0:0.1/10:1;
w = 2*pi*1;
a = 1;
phi = 2*pi/numofjoints;
all_angles=[];
for jointidx = jointidcs
    all_angles = [all_angles;a * sin(w*t-jointidx*phi)];
end

func = @(x) colorspace('RGB->Lab',x);
colors = distinguishable_colors(8,'w',func);
    
figure;
hold on;
plot(t,all_angles(1,:),'k','LineWidth',20);
ylabel('$\phi_0$','Interpreter','latex','Fontsize',40);
xlabel('$t$','Interpreter','latex','Fontsize',40);
set(gca,'XTickLabel','','YTickLabel','');
ylim([-1.1 1.1]);
hold off
saveas(gcf,'jointangle0.emf');
%colors = jet(6);
for jointidx = 1:numofjoints-2
    figure;
    hold on;
    plot(t,all_angles(jointidx+1,:),'color',colors(jointidx,:),'LineWidth',20);
    plot(t,all_angles(jointidx+1,:),'color','k','LineWidth',5);
    str = sprintf('$\\phi_%d$',jointidx);
    ylabel(str,'Interpreter','latex','Fontsize',40);
    xlabel('$t$','Interpreter','latex','Fontsize',40);
    set(gca,'XTickLabel','','YTickLabel','');
    ylim([-1.1 1.1]);
    fname = sprintf('jointangle%d.emf',jointidx);
    hold off;
    saveas(gcf,fname);
end
figure;
hold on;
plot(t,all_angles(numofjoints-1,:),'color',colors(8,:),'LineWidth',20);
plot(t,all_angles(numofjoints-1,:),'color','k','LineWidth',5);
ylabel('$\phi_7$','Interpreter','latex','Fontsize',40);
xlabel('$t$','Interpreter','latex','Fontsize',40);
set(gca,'XTickLabel','','YTickLabel','');
ylim([-1.1 1.1]);
hold off;
saveas(gcf,'jointangle7.emf');

