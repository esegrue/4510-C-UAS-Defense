function filename = adversaryPathsGenerator(dT, speed, turnRadius, elevationMap, targetPos, adv_height, filename, reset)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here

    arguments
        dT double
        speed double
        turnRadius double
        elevationMap map
        targetPos (1,2) double
        adv_height double
        filename char
        reset logical = false
    end
    
    % defining NFZs based on elevation
    mapL = elevationMap.size.vert;
    mapW = elevationMap.size.horiz;
    costmap = zeros(mapL+1, mapW+1);
    
    % FIXED: Assigned (Y,X) standard and safe getElevation to match simulator.m
    for y = 0:mapL
        for x = 0:mapW
            costmap(y+1,x+1) = elevationMap.getElevation(x,y) > (adv_height * 0.975); 
        end
    end
    
    % FIXED: Used flipud to correctly align with 3D map
    occMap = binaryOccupancyMap(flipud(costmap)); 
    
    ss = stateSpaceSE2;
    ss.StateBounds = [occMap.XWorldLimits; occMap.YWorldLimits; -pi pi];
    sv = validatorOccupancyMap(ss);
    sv.Map = occMap;
    
    figure(3)
    show(occMap)
    
    planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, "InterpolationDistance",speed*dT);
    
    entryBoundary = [occMap.XWorldLimits(1), occMap.XWorldLimits(1),occMap.YWorldLimits;
        occMap.XWorldLimits(2), occMap.XWorldLimits(2),occMap.YWorldLimits;
        occMap.XWorldLimits, occMap.YWorldLimits(1),occMap.YWorldLimits(1);
        occMap.XWorldLimits, occMap.YWorldLimits(2),occMap.YWorldLimits(2)];
    
    i = 1;
    hWaitbar = waitbar(0, 'New positions: 1', 'Name', 'Generating effector positions','CreateCancelBtn','delete(gcbf)');
    
    paths = {};
    
    while true
        side = entryBoundary(randi(4),:); %random side of the map of form [xlow xhigh ylow yhigh]
        entryPos = [side(1) + rand()*(side(2) - side(1)), side(3) + rand()*(side(4) - side(3))];
        entryHeading = atan2(targetPos(2) - entryPos(2), targetPos(1) - entryPos(1)); %starts out naively pointed towards target
        exitSide = entryBoundary(randi(4),:);
        exitPos = entryPos;
        exitHeading = pi+entryHeading;
        
        inPath = plan(planner, [entryPos, entryHeading], [targetPos, entryHeading+pi/2]);
        inStates = inPath.States; % [x y theta]
        outPath = plan(planner, inStates(end, :), [exitPos, exitHeading]);
        outStates = outPath.States;
        totPath = cat(1, inStates, outStates(2:end,:));

        % FIXED: Using proper cell array indexing to prevent concatenation errors
        paths{1, 1, end+1} = totPath;
        
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