function MA_analsis()
    %% Place 1avg_bodypos.out, 2avg_bodypos.out, 3avg_bodypos.out, 4avg_bodypos.out
    % and 1body_0_pos.out, 2body_0_pos.out, 3body_0_pos.out, 4body_0_pos.out
    % in this directory
    clear all;
    %close all;
    clear screen;
    set(0,'defaultfigurecolor',[1 1 1])
    targetvaluesxpos = [10, -10, 10, -10];
    targetvalueszpos = [10, 10, -10, -10];
    xtick = [-15:5:15];
    ytick = [-15:5:15];

    figure('units','normalized','outerposition',[0 0 1 1]);
    for i =1:4
        avgfname = sprintf('%davg_bodypos.out',i);
        inputfile = fopen(avgfname);
        comvalues = textscan(inputfile, '%f [%f, %f, %f]');
        fclose(inputfile);
        comtimestamp = comvalues{1};
        comxpos = comvalues{2};
        %ypos = values{3};
        comzpos = comvalues{4};

        bodyposfname = sprintf('%dbody_0_pos.out', i);
        inputfile = fopen(bodyposfname);
        headvalues = textscan(inputfile, '%f (%f, %f, %f)');
        fclose(inputfile);
        headvaluestimestamp = headvalues{1};
        headvaluesxpos = headvalues{2};
        %ypos = values{3};
        headvalueszpos = headvalues{4};
        subplot(2,2,i);
        hold on;
        grid on;
        titlestr = sprintf('Target @ X,Z (%.2f,%.2f)', targetvaluesxpos(i), targetvalueszpos(i));
        title(titlestr,'Fontsize',16);
        axis([-15 15 -15 15])
        plot(comxpos, comzpos, '.b', 'Markersize', 5);
        plot(headvaluesxpos, headvalueszpos, '.r', 'Markersize', 5);
        plot(targetvaluesxpos(i), targetvalueszpos(i),'.g', 'Markersize', 20);
        xlabel('X','Fontsize',15)
        ylabel('Z','Fontsize',15)
        l = legend('Avg body position','Head position','Target');
        set(l,'Location','best');
        set(gca,'XTick',xtick,'YTick',ytick,'Fontsize',12);
        hold off;
    end
    %fname = exportfname([],'_findtarget.pdf');
    %export_fig(fname,'-painters','-pdf', '-p5');
end

function fname=exportfname(prefix,postfix)
    curdir = pwd;
    dirs = textscan(curdir,'%s','Delimiter','\\');
    dirstr = dirs{1};
    fname = strcat(prefix,char(dirstr(end-2)),'_',char(dirstr(end-1)),postfix);
end
