function bodies = bodymovement(bodies, angles)
    assert((length(bodies)-1) == length(angles))
    %close all;
    %bodies = create_worm(1);
    %printworm(bodies);
    
%     [startX,endX,startY,endY] = getbodypositions(bodies{1})
%     %newendpoint = rotateclockwisepointY([startX,startY],degtorad(30),[endX,endY]);
%     newendpoint =rotatecounterclockwisepointY([startX,startY],degtorad(30),[endX,endY]);
%     newbody = [startX,newendpoint(1);startY,newendpoint(2)];

    %newbody = rotatebodyFrontRotPoint(bodies{1},degtorad(30));
    %newbody = rotatebodyBackRotPoint(bodies{1},degtorad(30));
    
    %bodies = {newbody};
    
    %numofbodies = 10;    
    %bodies = create_worm(numofbodies);
    %printworm(bodies);
    
    %[bodies{1},bodies{2}] = applyreljointangle(bodies{1},bodies{2},degtorad(10));
    %[bodies{1},bodies{2}] = applyreljointangle(bodies{1},bodies{2},degtorad(-10));
    % apply changes to bodies behind
%     for bodyidx=3:numofbodies
%         bodies{bodyidx} = alignbodytobasebody(bodies{bodyidx-1},bodies{bodyidx},degtorad(-10)/2);
%     end


%     [bodies{3},bodies{4}] = applyreljointangle(bodies{3},bodies{4},degtorad(-10));
%     % trailing bodies
%     trailingbodies = bodies(5:end);
%     trailingbodies = alignallfollowingbodies(bodies{4}, trailingbodies, degtorad(-10)/2);
%     bodies(5:end) = trailingbodies;
%     % leading bodies
%     leadingbodies = bodies(1:2);
%     leadingbodies = alignallleadingbodies(bodies{3}, leadingbodies, degtorad(-10)/2);
%     bodies(1:2) = leadingbodies;
    
    %bodies = applyanglebetweenbodies(3,4,bodies, degtorad(-10))
    %bodies = applyanglebetweenbodies(1,2,bodies, degtorad(-10))
    %bodies = applyanglebetweenbodies(numofbodies-1,numofbodies,bodies, degtorad(-10))
    
    %figure;
    %angles = degtorad(-9)*ones(1,numofbodies-1);
    %angles = degtorad(-8:2:9);
    for angleidx = 1:length(angles)
        bodyidx = angleidx;
        nextbodyidx = bodyidx+1;
        bodies = applyanglebetweenbodies(bodyidx,nextbodyidx,bodies, angles(angleidx));
        
        %printworm(bodies);
    end
    %figure;
    %printworm(bodies);
    

end

function bodies = applyanglebetweenbodies(bodyidx1,bodyidx2,bodies, theta)
    numofbodies = length(bodies);
    assert(bodyidx1 <= numofbodies && bodyidx1 >= 0);
    assert(bodyidx2 <= numofbodies && bodyidx2 >= 0);
    
    CoM_before = calcCenterofMass(bodies);
    
    trailingbodies = bodies(bodyidx2+1:end);
    leadingbodies = bodies(1:bodyidx1-1);
    % set angle
    % fluid dynamic
    relangle2 = theta * (length(leadingbodies)+1)/numofbodies; % angle proportion for body 2
    relangle1 = theta * (length(trailingbodies)+1)/numofbodies; % angle proportion for body 1
    % no fluid dynamic
    %relangle2 = theta;
    %relangle1 = 0;
    [bodies{bodyidx1},bodies{bodyidx2}] = applyreljointangle(bodies{bodyidx1},bodies{bodyidx2},relangle1,relangle2);

    % adjust all trailing bodies    
    %trailingbodies = alignallfollowingbodies(bodies{bodyidx2}, trailingbodies, theta/2);
    trailingbodies = alignallfollowingbodies(bodies{bodyidx2}, trailingbodies, relangle2);
    bodies(bodyidx2+1:end) = trailingbodies; % store adjusted bodies

    % adjust all leading bodies    
    %leadingbodies = alignallleadingbodies(bodies{bodyidx1}, leadingbodies, theta/2);
    leadingbodies = alignallleadingbodies(bodies{bodyidx1}, leadingbodies, relangle1);
    bodies(1:bodyidx1-1) = leadingbodies;
    
    CoM_after = calcCenterofMass(bodies);
    bodies = normalizebyCoM(bodies,CoM_before,CoM_after);
end

function bodies = alignallleadingbodies(basebody, leadingbodies, theta)
    numofleadingbodies = length(leadingbodies);
    bodies = {};
    lastbody = basebody;
    for bodyidx=numofleadingbodies:-1:1
        bodies{bodyidx} = alignbodytobasebody_leading(lastbody,leadingbodies{bodyidx},theta);
        lastbody = bodies{bodyidx};
    end
end

function body = alignbodytobasebody_leading(basebody,body,theta)
    % rotate
    body = rotatebodyBackRotPoint(body,theta);
    
    % translate
    [body_startX,body_endX,body_startY,body_endY] = getbodypositions(body); % need for deciding which way to translate
    [startX,endX,startY,endY] = getbodypositions(basebody);
    %if body_startY > base_startY
    %    body = body - [0,0;base_startY,base_startY];
    %else
    %    body = body + [0,0;base_startY,base_startY];
    %end
    body = body + [startX-body_endX,startX-body_endX;startY-body_endY,startY-body_endY];

end


function bodies = alignallfollowingbodies(basebody, trailingbodies, theta)
    numoftrailingbodies = length(trailingbodies);
    bodies = {};
    lastbody = basebody;
    for bodyidx=1:numoftrailingbodies
        bodies{bodyidx} = alignbodytobasebody_trailing(lastbody,trailingbodies{bodyidx},theta);
        lastbody = bodies{bodyidx};
    end
end

function body = alignbodytobasebody_trailing(basebody,body,theta)
    % rotate
    body = rotatebodyFrontRotPoint(body,theta);
    % translate
    [body_startX,body_endX,body_startY,body_endY] = getbodypositions(body); % need for deciding which way to translate
    [startX,endX,startY,endY] = getbodypositions(basebody);
    body = body + [endX-body_startX,endX-body_startX;endY-body_startY,endY-body_startY];
end

function printworm(bodies)
    numofbodies = length(bodies);
    %figure;
    hold on;
    
    bidx = 1;
    [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
    line([startX endX],[startY endY],'LineStyle','-');
    for bidx = 2:numofbodies-1
        [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
        line([startX endX],[startY endY],'Marker','.','LineStyle','-');
    end
    bidx = numofbodies;
    [startX,endX,startY,endY] = getbodypositions(bodies{bidx});
    line([startX endX],[startY endY],'LineStyle','-');
    
    hold off;
end

function [startX,endX,startY,endY] = getbodypositions(body)
    startX = body(1,1);
    endX = body(1,2);
    startY = body(2,1);
    endY = body(2,2);
end

function point = rotateclockwisepointY(rotationpoint, thetaY, point)
    % translate point relative to rotation point
    tpoint = point - rotationpoint;
    tpoint = tpoint';
    % calc rotation
    rotmatrixY = [cos(thetaY)  -sin(thetaY) ; sin(thetaY)  cos(thetaY)];
    rpoint = rotmatrixY * tpoint;
    
    % translate back
    point = rpoint' + rotationpoint;
end

function point = rotatecounterclockwisepointY(rotationpoint, thetaY, point)
    point = rotateclockwisepointY(rotationpoint, -thetaY, point);
end

function body = rotatebodyFrontRotPoint(body,theta)
    [startX,endX,startY,endY] = getbodypositions(body);
    newendpoint = rotatecounterclockwisepointY([startX,startY],theta,[endX,endY]);
    body = [startX,newendpoint(1);startY,newendpoint(2)];
end

function body = rotatebodyBackRotPoint(body,theta)
    [startX,endX,startY,endY] = getbodypositions(body);
    newendpoint = rotateclockwisepointY([endX,endY],theta,[startX,startY]);
    body = [newendpoint(1),endX;newendpoint(2),endY];
end

function [body1,body2] = applyreljointangle(body1,body2,reljointangle1, reljointangle2) 
    %jointangle1 = reljointangle/2;
    body1 = rotatebodyBackRotPoint(body1,reljointangle1);
    %jointangle2 = reljointangle/2;
    body2 = rotatebodyFrontRotPoint(body2,reljointangle2);
end

function CoM = calcCenterofMass(bodies)

    x = [];
    y = [];
    for bodyidx = 1:length(bodies)
        x = [x, bodies{bodyidx}(1,1), bodies{bodyidx}(1,2)]; %add start and end value
        y = [y, bodies{bodyidx}(2,1), bodies{bodyidx}(2,2)];
    end
    % X
    % Y
    CoM = [mean(x);mean(y)];
end

function bodies = normalizebyCoM(bodies, CoM_before, CoM_after)
    translateX = CoM_before(1) - CoM_after(1);
    translateY = CoM_before(2) - CoM_after(2);
    
    for bodyidx = 1:length(bodies)
        bodies{bodyidx}(1,1) = bodies{bodyidx}(1,1) + translateX;
        bodies{bodyidx}(1,2) = bodies{bodyidx}(1,2) + translateX;
        bodies{bodyidx}(2,1) = bodies{bodyidx}(2,1) + translateY;
        bodies{bodyidx}(2,2) = bodies{bodyidx}(2,2) + translateY;
    end
end