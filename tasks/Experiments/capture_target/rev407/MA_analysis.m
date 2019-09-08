function MA_analsis()
    %% Place 1avg_bodypos.out, 2avg_bodypos.out, 3avg_bodypos.out, 4avg_bodypos.out
    % and 1body_0_pos.out, 2body_0_pos.out, 3body_0_pos.out, 4body_0_pos.out
    % in this directory
    clear all;
    %close all;
    clear screen;
    set(0,'defaultfigurecolor',[1 1 1])
    %xtick = [-60:10:60];
    %ytick = [-60:10:60];

    figure('units','normalized','outerposition',[0 0 1 1]);
    for i =1:4
        targetfname = sprintf('%dtargetpos.out',i);
        inputfile = fopen(targetfname);
        comvalues = textscan(inputfile, '%f (%f, %f, %f)');
        fclose(inputfile);
        targettimestamp = comvalues{1};
        targetxpos = comvalues{2};
        %ypos = values{3};
        targetzpos = comvalues{4};

        
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
        titlestr = sprintf('Target @ X,Z (%.2f,%.2f)', targetxpos(1), targetzpos(1));
        title(titlestr,'Fontsize',16);
        axis([-40 40 -40 40])
        plot(targetxpos(1:20:end), targetzpos(1:20:end), '.g', 'Markersize', 20);
        plot(comxpos(1:20:end), comzpos(1:20:end), '.b', 'Markersize', 5);
        plot(headvaluesxpos(1:20:end), headvalueszpos(1:20:end), '.r', 'Markersize', 5);
        xlabel('X','Fontsize',15)
        ylabel('Z','Fontsize',15)
        l = legend('Avg body position','Head position','Target');
        set(l,'Location','best');
        %set(gca,'XTick',xtick,'YTick',ytick,'Fontsize',12);
        set(gca,'Fontsize',12);
        hold off;
    end
    fname = exportfname([],'.pdf');
    export_fig(fname,'-painters','-pdf', '-p5');
end

function fname=exportfname(prefix,postfix)
    curdir = pwd;
    dirs = textscan(curdir,'%s','Delimiter','\\');
    dirstr = dirs{1};
    fname = strcat(prefix,char(dirstr(end-1)),'_',char(dirstr(end)),postfix);
end
