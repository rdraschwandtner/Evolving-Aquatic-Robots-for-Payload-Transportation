function printworm(bodies, fignum)
    numofbodies = length(bodies);
    %figure(fignum);
    hold on;
    
    grey = [0.3,0.3,0.3];
    
    bidx = 1;
    [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
    line([startX endX],[startY endY],'LineStyle','-','LineWidth',3,'Color',[0,0,0]);
    for bidx = 2:numofbodies-1
        [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
        line([startX endX],[startY endY],'Marker','.','markers',25,...
            'MarkerEdgeColor',grey,'LineStyle','-','LineWidth',3,...
            'Color',[0,0,0]);
    end
    bidx = numofbodies;
    [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
    line([startX endX],[startY endY],'LineStyle','-','LineWidth',3,'Color',[0,0,0]);
    
    plot(startX,startY,'.','Color',grey,'MarkerSize',25,'MarkerFaceColor',grey);  %# Plot a red dot
    hold off;
end

function [startX,endX,startY,endY] = getbodypositions(body)
    startX = body(1,1);
    endX = body(1,2);
    startY = body(2,1);
    endY = body(2,2);
end