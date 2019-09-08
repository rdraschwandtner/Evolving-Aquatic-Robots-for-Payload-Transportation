function stopping_bodylength()
    clear all
    close all
    
    densities = [50, 500, 25, 12, 8, 200]
    %densities = [50];
    legendstr = {};
    a = 1;
    for d=densities
        bodyfilename = sprintf('d%dbody.out', d);
        bodyfile = fopen(bodyfilename);
        bvalues = textscan(bodyfile, 'length:%f;density:%f;dims:[%f, %f, %f];numofsegs:%d');
        fclose(bodyfile);
        blength=bvalues{1};
        density=bvalues{2};
        dims=[bvalues{3};bvalues{4};bvalues{5}];
        numofsegs = bvalues{6};

        volume = dims(1)*dims(2)*dims(3);
        totalmass = numofsegs*density*volume
        currentlegendstr = sprintf('mass=%.1f (kg)', totalmass);
        legendstr{a} = currentlegendstr;

        positionfilename = sprintf('d%davg_bodypos.out', d);
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
        title('distance progress in body lengths');
        xlabel('time');
        ylabel('body lengths');
        grid on;
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
        title('veloctiy progress');
        xlabel('time');
        ylabel('velocity (body lengths/sec)');
        grid on;
        hold off;

        distanceprogress = distanceprogress(2:end); %avoid division by zero
        comtimestamp = comtimestamp(2:end); %avoid division by zero
        avg_vprogress = distanceprogress./comtimestamp;
        figure(3);
        hold all;
        plot(comtimestamp, avg_vprogress, '.-', 'Markersize', 5);
        title('average veloctiy progress');
        xlabel('time');
        ylabel('velocity (absolute body lengths/absolute secs)');
        grid on;
        hold off;
        a = a + 1;
    end
   
    
    
    figure(1);
    hold on;
    legend(legendstr);
    hold off;
    figure(2);
    hold on;
    legend(legendstr);
    hold off;
    figure(3);
    hold on;
    legend(legendstr);
    hold off;
    
end

function val = eucdistance(v1, v2)
    val = sqrt(sum((v1 - v2) .^ 2));
end