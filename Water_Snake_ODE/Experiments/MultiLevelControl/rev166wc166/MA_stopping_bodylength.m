function MA_stopping_bodylength()
    clear all
    close all
    
    set(0,'defaultfigurecolor',[1 1 1])
    
    frequencies = {'02'; '04';'06';'10';'20';'30';'40'};
    frequencies_lstring = {'0.2'; '0.4';'0.6';'1.0';'2.0';'3.0';'4.0'};
    %densities = [50];
    legendstr = {};
    a = 1;
    for fidx=1:length(frequencies)
        bodyfilename = sprintf('f%sbody.out', frequencies{fidx});
        bodyfile = fopen(bodyfilename);
        bvalues = textscan(bodyfile, 'length:%f;density:%f;dims:[%f, %f, %f];numofsegs:%d');
        fclose(bodyfile);
        blength=bvalues{1};
        density=bvalues{2};
        dims=[bvalues{3};bvalues{4};bvalues{5}];
        numofsegs = bvalues{6};

        volume = dims(1)*dims(2)*dims(3);
        totalmass = numofsegs*density*volume
        massstr = sprintf('Mass = %.1f (untis)', totalmass);
        
        currentlegendstr = sprintf('%s (Hz)', frequencies_lstring{fidx});
        legendstr{a} = currentlegendstr;

        positionfilename = sprintf('f%savg_bodypos.out', frequencies{fidx});
        inputfile = fopen(positionfilename);
        comvalues = textscan(inputfile, '%f [%f, %f, %f]');
        fclose(inputfile);
        comtimestamp = comvalues{1};
        comxpos = comvalues{2}./blength;
        %ypos = values{3};
        comzpos = comvalues{4}./blength;

        startposx = comxpos(1);
        startposz = comzpos(1);
        startvec = [startposx;startposz];

        distanceprogress = [];
        for i=1: length(comtimestamp)
            vec1 = [comxpos(i);comzpos(i)];
            distanceprogress = [distanceprogress; eucdistance(vec1, startvec)];
        end
        figure(1);
        hold all;
        plot(comtimestamp, distanceprogress, '.-', 'Markersize', 5);
        title('Distance progress','Fontsize',17);
        xlabel('Time (s)','Fontsize',15);
        ylabel('Body lengths','Fontsize',15);
        hold off;

        distancepertimestep = [];
        for i=1: length(comtimestamp)-1
            vec1 = [comxpos(i);comzpos(i)];
            vec2 = [comxpos(i+1);comzpos(i+1)];
            distancepertimestep = [distancepertimestep; eucdistance(vec1, vec2)];
        end

        vprogress = distancepertimestep./(comtimestamp(2)-comtimestamp(1));
        figure(2);
        hold all;
        plot(comtimestamp(2:end), vprogress, '.-', 'Markersize', 5);
        title('Veloctiy progress','Fontsize',17);
        xlabel('Time (s)','Fontsize',15);
        ylabel('Body lengths/s','Fontsize',15);
        hold off;
        a = a + 1;
    end   
    legendstr{a} = 'Stop signal';
    figure(1);
    hold on;
    line([10 10],[0 9],'Color','r');
    set(gca,'Fontsize',12);
    l = legend(legendstr);
    set(l,'location','best');
    hold off;
    %fname = exportfname([],'_distprog_freqs.pdf');
    %export_fig(fname,'-painters','-pdf', '-p5');
    figure(2);
    hold on;
    line([10 10],[0 0.7],'Color','r');
    set(gca,'Fontsize',12);
    l = legend(legendstr);
    set(l,'location','best');
    hold off;
    fname = exportfname([],'_velprog_freq.pdf');
    export_fig(fname,'-painters','-pdf', '-p5');
    
end

function val = eucdistance(v1, v2)
    val = sqrt(sum((v1 - v2) .^ 2));
end

function fname=exportfname(prefix,postfix)
    curdir = pwd;
    dirs = textscan(curdir,'%s','Delimiter','\\');
    dirstr = dirs{1};
    fname = strcat(prefix,char(dirstr(end-1)),'_',char(dirstr(end)),postfix);
end