classdef simulator < handle
    properties
        map
        UAS
        UASPos_all
        effectors
        sensors
        asset
        tick
        dt
        tps
        animate
        NFZs
        resetGraphics
        animationMultiplier
        hideClock
        fadePings
        costConfig
        effectors3D 
        occMap
    end

    methods
        function obj = simulator(map, uas, effectors, sensors, asset, options)
            arguments
                map, uas, effectors, sensors, asset
                options.tps = 20
                options.animate = true
                options.nfzs = polyshape.empty
                options.resetGraphics = true
                options.animationMultiplier = 1
                options.hideClock = false
                options.fadePings = false
                options.costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250)
            end
            obj.map = map; obj.UAS = uas; obj.effectors = effectors; obj.sensors = sensors; obj.asset = asset;
            obj.tick = 0; obj.tps = options.tps; obj.dt = 1 / obj.tps;
            obj.animate = options.animate; obj.NFZs = options.nfzs; obj.resetGraphics = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier; obj.hideClock = options.hideClock; obj.fadePings = options.fadePings; 
            obj.costConfig = options.costConfig;

            % Build occupancy map from terrain elevation
            mapL = obj.map.size.vert;
            mapW = obj.map.size.horiz;
            costmap = zeros(mapL+1, mapW+1);  % rows=y, cols=x
            if ~isempty(obj.UAS) && isprop(obj.UAS(1), 'altitude')
                flightAlt = obj.UAS(1).altitude;
            else
                flightAlt = 25; 
            end
            for x = 0:1:mapW
                for y = 0:1:mapL
                    costmap(y+1, x+1) = obj.map.getElevation(x,y) > flightAlt;
                end
            end

            % Burn NFZ polyshapes into costmap
            if ~isempty(obj.NFZs)
                for x = 0:1:mapW
                    for y = 0:1:mapL
                        if costmap(y+1, x+1) == 0
                            for n = 1:length(obj.NFZs)
                                if isinterior(obj.NFZs(n), x, y)
                                    costmap(y+1, x+1) = 1;
                                    break;
                                end
                            end
                        end
                    end
                end
            end

            obj.occMap = binaryOccupancyMap(flipud(costmap));

            % Initialize history for N UAS
            obj.UASPos_all = cell(1, length(obj.UAS));
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
            end
        
            if ~isempty(obj.effectors)
                numEff = length(obj.effectors);
                obj.effectors3D = zeros(numEff, 3);
                for k = 1:numEff
                    loc = obj.effectors(k).location;
                    z = obj.map.getElevation(loc(1), loc(2));
                    obj.effectors3D(k, :) = [loc(1), loc(2), z];
                end
            end
        end

        function results = runSim(obj)
            dt_local = obj.dt;
            cost_eff = obj.costConfig.effector; 
            cost_leak = obj.costConfig.leak; 
            cost_asset = obj.costConfig.asset;
            
            hasSensors = ~isempty(obj.sensors);
            if hasSensors
                p = [obj.sensors.params]; 
                sensorD50 = [p.d50]; sensorK = [p.k];
                req_pings = p(1).pings; 
                scan_rate = p(1).scanRate;
                sensorLocs = reshape([obj.sensors.location], 2, [])'; 
            else
                scan_rate = 1; req_pings = 1;
            end

            hasEffectors = ~isempty(obj.effectors3D);
            if hasEffectors
                effRanges = [obj.effectors.range]';
            end
            
            hasAsset = ~isempty(obj.asset);
            if hasAsset
                assetLoc = obj.asset.location; 
            end
            
            terrainProxy = obj.map.terrainProxy;
            numUAS = length(obj.UAS);
            uas_active = true(numUAS, 1);
            
            track_hist = zeros(numUAS, req_pings); 

            detectionData = cell(numUAS, 1);
            for i = 1:numUAS
                detectionData{i} = [];
            end

            isAssetDestroyed = false; cost = 0; UASkilled = 0; UASkillLocations = []; outcomeLog = strings(0); 
            
            animate_on = obj.animate;
            if animate_on
                if obj.resetGraphics; obj.map.wipeAnimation(); end
                obj.map.startAnimation(obj.asset, obj.effectors, obj.sensors, numUAS, obj.hideClock, obj.NFZs);
                UASsensedPos = [];
                view(0,90)
            end
            
            simComplete = false; tick_count = 0;
            max_expected_ticks = 400;
            if animate_on
                for i = 1:numUAS
                    obj.UASPos_all{i} = [obj.UASPos_all{i}; NaN(max_expected_ticks, 3)];
                end
            end

            while ~simComplete
                simComplete = true; 
                tick_count = tick_count + 1;

                if tick_count > max_expected_ticks
                    break;
                end

                obj.tick = tick_count;
                currentTime = tick_count * dt_local;
                
                timeSinceLastScan = mod(currentTime, scan_rate);
                isScanTick = (timeSinceLastScan < dt_local/2) || (abs(timeSinceLastScan - scan_rate) < dt_local/2);
                
                targetFound = false; targetPos = []; targetObj = [];

                % 1. SENSE & MOVE UAS
                for i = 1:numUAS
                    if ~uas_active(i); continue; end
                    simComplete = false;
                    
                    uasObj = obj.UAS(i);
                    if uasObj.mode == "Linear"
                        uasObj.linearMotion(dt_local);
                    elseif uasObj.mode == "HybridAStar"
                        uasObj.hybridAStarMotion(dt_local, tick_count, uasObj.turnRadius, obj.occMap)
                    elseif uasObj.mode == "Search"
                        uasObj.searchMotion(dt_local, obj.asset, isAssetDestroyed, obj.NFZs);
                    end
                    pos = uasObj.position;
                    
                    if animate_on
                        target_idx = tick_count + 1;
                        if target_idx > size(obj.UASPos_all{i}, 1)
                            obj.UASPos_all{i} = [obj.UASPos_all{i}; NaN(1000,3)];
                        end
                        obj.UASPos_all{i}(target_idx,:) = pos;
                    end
                    
                    isTracked = false; isPinged = false;
                    if hasSensors && isScanTick
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + (sensorLocs(:,2) - pos(2)).^2);
                        raw_probs = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));

                        % NFZ line-of-sight blocking
                        nSens = size(sensorLocs, 1);
                        for s = 1:nSens
                            if raw_probs(s) <= 0; continue; end
                            if simulator.losBlockedByNFZ(sensorLocs(s,:), pos(1:2), obj.NFZs)
                                raw_probs(s) = 0;
                            end
                        end

                        % Uphill terrain penalty
                        for s = 1:nSens
                            if raw_probs(s) <= 0; continue; end
                            sElev = terrainProxy(sensorLocs(s,2), sensorLocs(s,1));
                            maxElev = simulator.maxTerrainOnPath(sensorLocs(s,:), pos(1:2), terrainProxy);
                            if maxElev > sElev
                                penalty = max(0, 1 - (maxElev - sElev) / d_sens(s));
                                raw_probs(s) = raw_probs(s) * penalty;
                            end
                        end

                        probs = min(raw_probs, 0.90);
                        if any(probs >= rand(size(probs))); isPinged = true; end
                        track_hist(i, :) = [track_hist(i, 2:end), isPinged];
                        detectionData{i} = [detectionData{i}; pos(1:2), max(probs), isPinged];
                    end

                    if sum(track_hist(i,:)) >= req_pings; isTracked = true; end
                    
                    if isPinged && animate_on
                         UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                         obj.map.animateUASsensed(UASsensedPos);
                    end

                    % Target designation for mobile effectors (Grabs closest tracked)
                    if isTracked && ~targetFound
                        targetFound = true;
                        targetPos = pos(1:2);
                        targetObj = uasObj;
                    end
                end 

                % 2. MOVE EFFECTORS
                if targetFound && hasEffectors
                    obj.stepEffectors_(targetPos, targetObj);
                    
                    % Update 3D array for collision checking
                    for k = 1:length(obj.effectors)
                        loc = obj.effectors(k).location;
                        z = terrainProxy(loc(2), loc(1)); 
                        obj.effectors3D(k, :) = [loc(1), loc(2), z];
                    end
                end
                
                % 3. CHECK COLLISIONS
                for i = 1:numUAS
                    if ~uas_active(i); continue; end
                    pos = obj.UAS(i).position;
                    uasObj = obj.UAS(i);

                    eventEffector = false;
                    if sum(track_hist(i,:)) >= req_pings && hasEffectors
                        effLocs = obj.effectors3D;
                        d_eff = sqrt(sum((effLocs - pos).^2, 2));
                        if any(d_eff <= effRanges); eventEffector = true; end
                    end
                    
                    eventAsset = false;
                    if hasAsset
                        d_asset = sqrt((assetLoc(1) - pos(1)).^2 + (assetLoc(2) - pos(2)).^2);
                        if d_asset <= (uasObj.speed * dt_local); eventAsset = true; end
                    end
                    
                    z_terr = terrainProxy(pos(2), pos(1)); 
                    eventCrash = (pos(3) <= z_terr);
                    eventExit = tick_count > 10 && ((pos(1) <= 0) || (pos(1) >= obj.map.size.horiz) || (pos(2) <= 0) || (pos(2) >= obj.map.size.vert));
                    
                    if eventEffector
                        cost = cost + cost_eff*1/d_asset*currentTime; 
                        outcomeLog(end+1) = "Intercept";
                        UASkillLocations = [UASkillLocations; pos];
                        uasObj.active = false; uas_active(i) = false; 
                        UASkilled = UASkilled + 1;
                        if animate_on; obj.map.animateUASkilled(pos); end
                        
                    elseif eventCrash
                        cost = cost + cost_leak; outcomeLog(end+1) = "TerrainCrash";
                        uasObj.active = false; uas_active(i) = false;
                        if animate_on; obj.map.animateUAScrashed(pos); end
                        
                    elseif eventExit
                        cost = cost + cost_leak; outcomeLog(end+1) = "Escaped";
                        uasObj.active = false; uas_active(i) = false;
                        
                    elseif eventAsset
                        if ~isAssetDestroyed
                            isAssetDestroyed = true;
                            cost = cost + cost_asset; outcomeLog(end+1) = "AssetHit";
                            if animate_on; obj.map.animateDestroyedAsset(obj.asset); end
                        end
                    end
                end
                
                % 4. UPDATE ANIMATION
                if animate_on
                    pause(dt_local/obj.animationMultiplier);
                    obj.map.updateUASAnimation(obj.UASPos_all);
                    if hasEffectors; obj.map.updateEffectors(obj.effectors); end
                    if ~obj.hideClock; obj.map.updateClock(currentTime); end
                end
            end
            
            if animate_on
                for i = 1:numUAS
                    valid_rows = ~isnan(obj.UASPos_all{i}(:,1));
                    obj.UASPos_all{i} = obj.UASPos_all{i}(valid_rows,:);
                end
            end

            results.UASPos_all = obj.UASPos_all;
            results.isAssetDestroyed = isAssetDestroyed; 
            results.cost = cost; 
            results.UASkilled = UASkilled;
            results.UASkillLocations = UASkillLocations;
            results.outcomeLog = outcomeLog;
            results.tick = tick_count;
            results.detectionData = detectionData;
        end
    end

    methods (Access = private)

        function stepEffectors_(obj, targetPosXY, targetObj)
            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            for e = 1:numel(obj.effectors)
                eff = obj.effectors(e);
                if eff.mode == "STATIC"; continue; end

                step = eff.speed * obj.dt;
                replanEveryTicks = 30;
                replanDist = 5.0;

                interceptPose = obj.predictIntercept_(targetPosXY, targetObj);

                if isempty(eff.planner)
                    ss = stateSpaceSE2;
                    ss.StateBounds = [xMin xMax; yMin yMax; -pi pi];
                    sv = validatorOccupancyMap(ss);
                    sv.Map = obj.occMap;
                    sv.ValidationDistance = 0.5;
                    eff.planner = plannerHybridAStar(sv, 'MinTurningRadius', 3.0, "InterpolationDistance", step);
                end

                if all(isfinite(eff.lastInterceptPose))
                    interceptMoved = norm(interceptPose(1:2) - eff.lastInterceptPose(1:2)) >= replanDist;
                else
                    interceptMoved = true;
                end

                needReplan = (isempty(eff.path) || (obj.tick - eff.lastPlanTick) >= replanEveryTicks || interceptMoved);

                if needReplan
                    startPose = [eff.location eff.heading];
                    try
                        pathObj = plan(eff.planner, startPose, interceptPose);
                        if isprop(pathObj, "States") && ~isempty(pathObj.States)
                            eff.path = pathObj.States;
                            eff.pathIdx = 2; 
                        else
                            eff.path = []; eff.pathIdx = 1;
                        end
                    catch
                        eff.path = []; eff.pathIdx = 1;
                    end
                    eff.lastPlanTick = obj.tick;
                    eff.lastInterceptPose = interceptPose;
                end

                moved = false;
                if ~isempty(eff.path)
                    idx = max(1, min(eff.pathIdx, size(eff.path, 1)));
                    nextPose = eff.path(idx, :);

                    if all(isfinite(nextPose))
                        eff.location = nextPose(1:2);
                        eff.heading = nextPose(3);
                        eff.pathIdx = eff.pathIdx + 1;
                        if eff.pathIdx > size(eff.path, 1)
                            eff.path = []; eff.pathIdx = 1;
                        end
                        moved = true;
                    else
                        eff.path = [];
                    end
                end

                if ~moved
                    goal = interceptPose(1:2);
                    v = goal - eff.location;
                    nv = norm(v);
                    if nv > 1e-9
                        dir = v / nv;
                        newLoc = eff.location + step * dir;
                        newLoc(1) = min(max(newLoc(1), xMin), xMax);
                        newLoc(2) = min(max(newLoc(2), yMin), yMax);
                        eff.location = newLoc;
                        eff.heading = atan2(dir(2), dir(1));
                    end
                end
                obj.effectors(e) = eff;
            end
        end

        function interceptPose = predictIntercept_(~, targetPosXY, targetObj)
            lookahead = 2.0; vhat = [1 0];
            if isprop(targetObj, "targetUnitVector")
                v = targetObj.targetUnitVector(1:2);
                if norm(v) > 0; vhat = v / norm(v); end
            end
            spd = 0;
            if isprop(targetObj, "speed"); spd = targetObj.speed; end
            ip = targetPosXY + vhat * spd * lookahead;
            hdg = atan2(vhat(2), vhat(1));
            interceptPose = [ip, hdg];
        end
    end

    methods (Static, Access = private)
        function blocked = losBlockedByNFZ(sensorXY, targetXY, nfzArray)
            % Check if the line segment from sensor to target intersects any NFZ
            blocked = false;
            if isempty(nfzArray); return; end
            nSteps = 20;
            for t = linspace(0, 1, nSteps)
                pt = sensorXY + t * (targetXY - sensorXY);
                for n = 1:length(nfzArray)
                    if isinterior(nfzArray(n), pt(1), pt(2))
                        blocked = true;
                        return;
                    end
                end
            end
        end

        function maxZ = maxTerrainOnPath(sensorXY, targetXY, terrainProxy)
            % Sample terrain elevation along the line from sensor to target
            nSteps = 10;
            maxZ = -inf;
            for t = linspace(0, 1, nSteps)
                pt = sensorXY + t * (targetXY - sensorXY);
                z = terrainProxy(pt(2), pt(1));
                if z > maxZ; maxZ = z; end
            end
        end
    end
end