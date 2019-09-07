function bodies = create_worm(numberofsegments, segmentlen, startpos)
    startpos = [0,0]; %[X,Y]
    segmentlen = 1.5;
    
    bodylayout = [0,segmentlen;0,0];
    
    bodies = {};
    initbody = [0,-segmentlen;0,0];
    bodies{1} = initbody;
    lastbody = initbody;
    for bidx=2:numberofsegments       
        bodies{bidx} = lastbody - [segmentlen,segmentlen;0,0]; % in negative X direction
        lastbody = bodies{bidx};
    end
end