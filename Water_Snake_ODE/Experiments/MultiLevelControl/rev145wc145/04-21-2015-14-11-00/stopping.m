function stopping()
    %clear all
    %close all

    inputfile = fopen('avg_bodypos.out');
    comvalues = textscan(inputfile, '%f [%f, %f, %f]');
    fclose(inputfile);
    comtimestamp = comvalues{1};
    comxpos = comvalues{2};
    %ypos = values{3};
    comzpos = comvalues{4};

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
    title('euclidean distance progress');
    xlabel('time');
    ylabel('units');
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
    title('veloctiy per timestep progress');
    xlabel('time');
    ylabel('velocity (dt/timestep)');
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
    ylabel('velocity (absolute t/absolute time)');
    grid on;
    hold off;
    
    
    
end

function val = eucdistance(v1, v2)
    val = sqrt(sum((v1 - v2) .^ 2));
end