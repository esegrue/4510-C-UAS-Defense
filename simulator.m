classdef simulator
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

            %defining NFZs based on elevation
            mapL = obj.map.size.vert;
            mapW = obj.map.size.horiz;
            costmap = zeros(mapW+1, mapL+1);
            if ~isempty(obj.UAS) && isprop(obj.UAS(1), 'altitude')
                flightAlt = obj.UAS(1).altitude;
            else
                flightAlt = 25; % fallback
            end
            for x = 0:1:mapW
                for y = 0:1:mapL
                    costmap(x+1,y+1) = obj.map.getElevation(x,y) > flightAlt; %arbitrary height
                end
            end
            
            obj.occMap = binaryOccupancyMap(fliplr(costmap)); %map mirrors for some reason

            
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
                scan_rate = 1;
            end

            hasEffectors = ~isempty(obj.effectors3D);
            if hasEffectors
                effLocs = obj.effectors3D;
                effRanges = [obj.effectors.range]';
            end
            
            hasAsset = ~isempty(obj.asset);
            if hasAsset
                % Single asset location
                assetLoc = obj.asset.location; 
            end
            
            terrainProxy = obj.map.terrainProxy;
            numUAS = length(obj.UAS);
            uas_active = true(numUAS, 1);
            
            track_hist = zeros(numUAS, req_pings); 

            isAssetDestroyed = false; cost = 0; UASkilled = 0; UASkillLocations = []; outcomeLog = strings(0); 
            
            % Initialize Graphics
            animate_on = obj.animate;
            if animate_on
                if obj.resetGraphics
                    obj.map.wipeAnimation();
                end
                % Pass number of UAS to map (Removed AOR)
                obj.map.startAnimation(obj.asset, obj.effectors, obj.sensors, numUAS, obj.hideClock);
                UASsensedPos = [];
                view(0,90)
            end
            
            simComplete = false; tick_count = 0;
            max_expected_ticks = 10000;
            if animate_on
                for i = 1:numUAS
                    obj.UASPos_all{i} = [obj.UASPos_all{i}; NaN(max_expected_ticks, 3)];
                end
            end

            while ~simComplete
                simComplete = true; 
                tick_count = tick_count + 1;
                currentTime = tick_count * dt_local;
                
                % check sensor scan tick
                timeSinceLastScan = mod(currentTime, scan_rate);
                isScanTick = (timeSinceLastScan < dt_local/2) || (abs(timeSinceLastScan - scan_rate) < dt_local/2);
                
                for i = 1:numUAS
                    if ~uas_active(i)
                        continue;
                    end
                    simComplete = false;
                    
                    % 1. MOVE UAS
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
                    
                    % 2. CHECK SENSOR TRACKING
                    isTracked = false;
                    isPinged = false;
                    
                    if hasSensors && isScanTick
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + (sensorLocs(:,2) - pos(2)).^2);
                        raw_probs = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));
                        probs = min(raw_probs, 0.90); % Cap probability at 90%
                        
                        if any(probs >= rand(size(probs)))
                            isPinged = true;
                        end
                        
                        track_hist(i, :) = [track_hist(i, 2:end), isPinged];
                    end
                    
                    if sum(track_hist(i,:)) >= req_pings
                        isTracked = true;
                    end
                    
                    if isPinged && animate_on
                         UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                         obj.map.animateUASsensed(UASsensedPos);
                    end
                    
                    % 3. CHECK COLLISIONS
                    eventEffector = false;
                    if isTracked && hasEffectors
                        d_eff = sqrt(sum((effLocs - pos).^2, 2));
                        if any(d_eff <= effRanges); eventEffector = true; end
                    end
                    
                    eventAsset = false;
                    if hasAsset
                        d_asset = sqrt((assetLoc(1) - pos(1)).^2 + (assetLoc(2) - pos(2)).^2);
                        if d_asset <= (uasObj.speed * dt_local)
                            eventAsset = true;
                        end
                    end
                    
                    z_terr = terrainProxy(pos(2), pos(1)); 
                    eventCrash = (pos(3) <= z_terr);
                    
                    eventExit = tick_count > 10 && ((pos(1) <= 0) || (pos(1) >= obj.map.size.horiz) || (pos(2) <= 0) || (pos(2) >= obj.map.size.vert));
                    if eventEffector
                        cost = cost + cost_eff*1/d_asset*tick_count/100; 
                        outcomeLog(end+1) = "Intercept";
                        UASkillLocations = [UASkillLocations; pos];
                        uasObj.active = false; 
                        uas_active(i) = false; 
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
                            %simComplete = true; 
                        end
                    end
                end 
                
                % 3. UPDATE ANIMATION
                if animate_on
                    pause(dt_local/obj.animationMultiplier);
                    obj.map.updateUASAnimation(obj.UASPos_all);
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
        end
    end
end