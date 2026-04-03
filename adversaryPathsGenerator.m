function filename = adversaryPathsGenerator(dT, speed, turnRadius, elevationMap, targetPos, filename, reset)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here

    arguments
        dT double
        speed double
        turnRadius double
        elevationMap map
        targetPos (1,2) double
        filename char
        reset logical = false
    end
    
    %defining NFZs based on elevation
    mapL = elevationMap.size.vert;
    mapW = elevationMap.size.horiz;
    costmap = zeros(mapL+1, mapW+1);
    for x = 0:1:mapL
        for y = 0:1:mapW
            costmap(x+1,y+1) = elevationMap.getElevation(x,y) > 20; %arbitrary height
        end
    end
    
    occMap = binaryOccupancyMap(fliplr(costmap)); %map mirrors for some reason
    
    ss = stateSpaceSE2;
    ss.StateBounds = [occMap.XWorldLimits; occMap.YWorldLimits; -pi pi];
    sv = validatorOccupancyMap(ss);
    sv.Map = occMap;
    planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, "InterpolationDistance",speed*dT);
    
    
    entryBoundary = [occMap.XWorldLimits(1), occMap.XWorldLimits(1),occMap.YWorldLimits;
        occMap.XWorldLimits(2), occMap.XWorldLimits(2),occMap.YWorldLimits;
        occMap.XWorldLimits, occMap.YWorldLimits(1),occMap.YWorldLimits(1);
        occMap.XWorldLimits, occMap.YWorldLimits(2),occMap.YWorldLimits(2)];
    
    i = 1;
    hWaitbar = waitbar(0, 'New positions: 1', 'Name', 'Generating effector positions','CreateCancelBtn','delete(gcbf)');
    
    paths = {};
    
    while true
        entryIdx = randi(4);
side = entryBoundary(entryIdx,:);
entryPos = [side(1) + rand()*(side(2) - side(1)), side(3) + rand()*(side(4) - side(3))];

exitIdx = randi(4);
while exitIdx == entryIdx
    exitIdx = randi(4);
end
exitSide = entryBoundary(exitIdx,:);
exitPos  = [exitSide(1) + rand()*(exitSide(2) - exitSide(1)), ...
            exitSide(3) + rand()*(exitSide(4) - exitSide(3))];
        
        inPath = plan(planner, [entryPos, entryHeading], [targetPos, entryHeading+pi/2]);
        inStates = inPath.States; % [x y theta]
        outPath = plan(planner, inStates(end, :), [exitPos, exitHeading]);
        outStates = outPath.States;
        totPath = cat(1, inStates, outStates(2:end,:));

        paths = cat(3,paths, totPath);
        if ~ishandle(hWaitbar)
            disp('Generation stopped by user.')
            break
        else
            waitbar(i/(i+1000),hWaitbar, ['New paths: ' num2str(i)]);
            i = i + 1;
        end
        pause(0.01)
    end
    
    if isfile(filename)
        data = load(filename);
        if isfield(data, 'adversary_paths_bank') && (reset == false)
            adversary_paths_bank = data.('adversary_paths_bank');
            adversary_paths_bank = cat(3, adversary_paths_bank, paths);
        else
            adversary_paths_bank = paths;
        end
    else
        adversary_paths_bank = paths;
    end
    save(filename, 'adversary_paths_bank');
end


