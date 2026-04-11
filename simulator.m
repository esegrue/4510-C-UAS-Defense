classdef simulator < handle
    properties
        randomStream
        map
        UAS
        UASPos_all
        UASTrackedPos_all
        effectors
        sensors
        asset

        tick
        dt
        tps
        animate
        resetGraphics
        animationMultiplier
        hideClock
        fadePings
        costConfig
        effectors3D 
        occMap
        maxTicks

        directEnergyDwellTime
        kineticProjectileSpeed
        kineticShotsPerVolley
        kineticHitProbability
        kineticRefireTime
        projectileHitTolerance

        kineticFermiD50Scale
        kineticFermiSharpnessScale
        kineticUseFermiModel

        directEnergyTrackTime
        effectorLastFireTime
        projectiles
        projectileHandles
        projectileTrailHandles
        pendingKineticHits

        maxProjectiles
        numProjectiles
        maxPendingKineticHits
        numPendingKineticHits
    end

    methods
        function obj = simulator(map, uas, effectors, sensors, asset, options)
            arguments
                map, uas, effectors, sensors, asset
                options.randomStream = RandStream.getGlobalStream()
                options.tps = 20
                options.animate = true
                options.resetGraphics = true
                options.animationMultiplier = 1
                options.hideClock = false
                options.fadePings = false
                options.costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250)
                options.costmapMatrix = []
                options.maxTicks (1, 1) double = 10000
                
                options.directEnergyDwellTime (1, 1) double = 0.75
                options.kineticProjectileSpeed (1, 1) double = 30
                options.kineticShotsPerVolley (1, 1) double = 1
                options.kineticHitProbability (1, 1) double = 0.70
                options.kineticRefireTime (1, 1) double = 0.50
                options.projectileHitTolerance (1, 1) double = 0.75
                options.kineticFermiD50Scale (1, 1) double = 0.60
                options.kineticFermiSharpnessScale (1, 1) double = 0.12
                options.kineticUseFermiModel (1, 1) logical = true
            end
            obj.map = map; obj.UAS = uas; obj.effectors = effectors; obj.sensors = sensors; obj.asset = asset;
            obj.randomStream = options.randomStream;
            obj.tick = 0; obj.tps = options.tps; obj.dt = 1 / obj.tps;
            obj.animate = options.animate; obj.resetGraphics = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier; obj.hideClock = options.hideClock; obj.fadePings = options.fadePings; 
            obj.costConfig = options.costConfig;
            obj.maxTicks = options.maxTicks;

            obj.directEnergyDwellTime = max(0, options.directEnergyDwellTime);
            obj.kineticProjectileSpeed = max(eps, options.kineticProjectileSpeed);
            obj.kineticShotsPerVolley = max(1, round(options.kineticShotsPerVolley));
            obj.kineticHitProbability = min(1, max(0, options.kineticHitProbability));
            obj.kineticRefireTime = max(0, options.kineticRefireTime);
            obj.projectileHitTolerance = max(0.01, options.projectileHitTolerance);
            obj.kineticFermiD50Scale = max(0.01, options.kineticFermiD50Scale);
            obj.kineticFermiSharpnessScale = max(0.01, options.kineticFermiSharpnessScale);
            obj.kineticUseFermiModel = options.kineticUseFermiModel;

            if isempty(options.costmapMatrix)
                mapL = obj.map.size.vert;
                mapW = obj.map.size.horiz;
                if ~isempty(obj.UAS) && isprop(obj.UAS(1), 'altitude')
                    flightAlt = obj.UAS(1).altitude;
                else
                    flightAlt = 25; 
                end
                
                [X, Y] = meshgrid(0:mapW, 0:mapL);
                Z_raw = obj.map.terrainProxy(X', Y'); 
                Z = Z_raw'; 
                costmap = Z >= flightAlt;
                obj.occMap = binaryOccupancyMap(flipud(costmap));
            else
                obj.occMap = binaryOccupancyMap(options.costmapMatrix);
            end

            obj.UASPos_all = cell(1, length(obj.UAS));
            obj.UASTrackedPos_all = cell(1, length(obj.UAS)); % <--- INITIALIZE MAGENTA DATA
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
                obj.UASTrackedPos_all{i} = [NaN NaN NaN]; 
            end
        
            if ~isempty(obj.effectors)
                numEff = length(obj.effectors);
                obj.effectors3D = zeros(numEff, 3);
                for k = 1:numEff
                    loc = obj.effectors(k).location;
                    z = obj.map.getElevation(loc(1), loc(2));
                    obj.effectors3D(k, :) = [loc(1), loc(2), z + simulator.effectorHeight_()];
                end
            end

            obj.directEnergyTrackTime = zeros(length(obj.UAS), 1);
            obj.effectorLastFireTime = -inf(max(numel(obj.effectors), 1), 1);
            
            obj.maxProjectiles = 5000;
            obj.numProjectiles = 0;
            obj.maxPendingKineticHits = 5000;
            obj.numPendingKineticHits = 0;
            
            obj.projectiles = repmat(simulator.emptyProjectile_(), obj.maxProjectiles, 1);
            obj.pendingKineticHits = repmat(simulator.emptyPendingShot_(), obj.maxPendingKineticHits, 1);
            obj.projectileHandles = gobjects(0);
            obj.projectileTrailHandles = gobjects(0);
        end

        function results = runSim(obj)
            dt_local = obj.dt;
            cost_eff = obj.costConfig.effector; 
            cost_leak = obj.costConfig.leak; 
            cost_asset = obj.costConfig.asset;
            if isfield(obj.costConfig, 'maxRiskPenalty')
                risk_max = obj.costConfig.maxRiskPenalty;
                risk_decay = obj.costConfig.riskDecayRate;
            else
                risk_max = 0; risk_decay = 0;
            end

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
                effWeaponModes = string({obj.effectors.weaponMode})';
                legacyMask = (effWeaponModes == "LEGACY");
                deMask = (effWeaponModes == "DIRECT_ENERGY");
                kineticMask = (effWeaponModes == "KINETIC");
            else
                legacyMask = []; deMask = []; kineticMask = [];
            end
            
            hasAsset = ~isempty(obj.asset);
            if hasAsset
                assetLoc = obj.asset.location; 
            end
            
            terrainProxy = obj.map.terrainProxy;
            numUAS = length(obj.UAS);
            uas_active = true(numUAS, 1);
            
            track_hist = zeros(numUAS, req_pings); 

            isAssetDestroyed = false; cost = 0; UASkilled = 0; UASkillLocations = []; outcomeLog = strings(0); 
            
            animate_on = obj.animate;
            if animate_on
                if obj.resetGraphics; obj.map.wipeAnimation(); end
                obj.map.startAnimation(obj.asset, obj.effectors, obj.sensors, numUAS, obj.hideClock);
                UASsensedPos = [];
                view(0,90)
            end
            
            simComplete = false; tick_count = 0;
            max_expected_ticks = obj.maxTicks;
            if animate_on
                for i = 1:numUAS
                    obj.UASPos_all{i} = [obj.UASPos_all{i}; NaN(max_expected_ticks, 3)];
                    obj.UASTrackedPos_all{i} = [obj.UASTrackedPos_all{i}; NaN(max_expected_ticks, 3)]; % <--- PREALLOCATE MAGENTA ARRAY
                end
            end

            while ~simComplete && tick_count < obj.maxTicks
                simComplete = true; 
                tick_count = tick_count + 1;
                obj.tick = tick_count;
                currentTime = tick_count * dt_local;
                
                timeSinceLastScan = mod(currentTime, scan_rate);
                isScanTick = (timeSinceLastScan < dt_local/2) || (abs(timeSinceLastScan - scan_rate) < dt_local/2);
                
                targetFound = false; targetPos = []; targetObj = [];

                uasTracked = false(numUAS, 1);

                for i = 1:numUAS
                    if ~uas_active(i); continue; end
                    simComplete = false;
                    
                    uasObj = obj.UAS(i);
                    if uasObj.mode == "Linear"
                        uasObj.linearMotion(dt_local);
                    elseif uasObj.mode == "HybridAStar"
                        uasObj.hybridAStarMotion(dt_local, tick_count, uasObj.turnRadius, obj.occMap)
                    elseif uasObj.mode == "Search"
                        uasObj.searchMotion(dt_local, obj.asset, isAssetDestroyed);
                    end
                    pos = uasObj.position;
                    
                    if animate_on
                        target_idx = tick_count + 1;
                        if target_idx > size(obj.UASPos_all{i}, 1)
                            obj.UASPos_all{i} = [obj.UASPos_all{i}; NaN(1000,3)];
                            obj.UASTrackedPos_all{i} = [obj.UASTrackedPos_all{i}; NaN(1000,3)]; 
                        end
                        obj.UASPos_all{i}(target_idx,:) = pos;
                    end
                    
                    isTracked = false; isPinged = false;
                    if hasSensors && isScanTick
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + (sensorLocs(:,2) - pos(2)).^2);
                        raw_probs = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));
                        probs = min(raw_probs, 0.90); 
                        if any(probs >= rand(obj.randomStream, size(probs))); isPinged = true; end
                        track_hist(i, :) = [track_hist(i, 2:end), isPinged];
                    end
                    
                    if sum(track_hist(i,:)) >= req_pings
                        isTracked = true; 
                        uasTracked(i) = true;
                    end
                    
                    if animate_on
                        if isTracked
                            obj.UASTrackedPos_all{i}(target_idx,:) = pos;
                        else
                            obj.UASTrackedPos_all{i}(target_idx,:) = [NaN NaN NaN];
                        end
                    end

                    if isPinged && animate_on
                         UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                         obj.map.animateUASsensed(UASsensedPos);
                    end

                    if isTracked && ~targetFound
                        targetFound = true;
                        targetPos = pos(1:2);
                        targetObj = uasObj;
                    end
                end 

                if targetFound && hasEffectors
                    obj.stepEffectors_(targetPos, targetObj);
                    
                    for k = 1:length(obj.effectors)
                        if obj.effectors(k).mode == "MOBILE"
                            loc = obj.effectors(k).location;
                            z = terrainProxy(loc(1), loc(2)); 
                            obj.effectors3D(k, :) = [loc(1), loc(2), z + simulator.effectorHeight_()];
                        end
                    end
                end
                
                for i = 1:numUAS
                    if ~uas_active(i); continue; end
                    pos = obj.UAS(i).position;
                    uasObj = obj.UAS(i);

                    eventEffector = false;
                    
                    if uasTracked(i) && hasEffectors
                        
                        if any(legacyMask)
                            effLocs = obj.effectors3D(legacyMask, :);
                            rgs = effRanges(legacyMask);
                            d_eff = sqrt(sum((effLocs(:, 1:2) - pos(1:2)).^2, 2));
                            losOK = false(size(d_eff));
                            for ee = 1:numel(d_eff)
                                if all(isfinite(effLocs(ee, :)))
                                    losOK(ee) = obj.hasLineOfSight_(effLocs(ee, :), pos(1:3), simulator.losClearance_(), terrainProxy);
                                end
                            end
                            if any((d_eff <= rgs) & losOK)
                                eventEffector = true;
                            end
                        end
                        
                        if any(deMask) && ~eventEffector
                            effLocs = obj.effectors3D(deMask, :);
                            rgs = effRanges(deMask);
                            d_eff = sqrt(sum((effLocs(:, 1:2) - pos(1:2)).^2, 2));
                            inRangeMask = (d_eff <= rgs);
                            losMask = false(size(inRangeMask));
                            for ee = 1:numel(inRangeMask)
                                if inRangeMask(ee) && all(isfinite(effLocs(ee, :)))
                                    losMask(ee) = obj.hasLineOfSight_(effLocs(ee, :), pos(1:3), simulator.losClearance_(), terrainProxy);
                                end
                            end
                            if any(inRangeMask & losMask)
                                obj.directEnergyTrackTime(i) = obj.directEnergyTrackTime(i) + dt_local;
                            else
                                obj.directEnergyTrackTime(i) = 0;
                            end
                            if obj.directEnergyTrackTime(i) >= obj.directEnergyDwellTime
                                eventEffector = true;
                            end
                        elseif ~any(deMask) || eventEffector
                            obj.directEnergyTrackTime(i) = 0;
                        end

                    else
                        obj.directEnergyTrackTime(i) = 0;
                    end
                    
                    eventAsset = false;
                    if hasAsset
                        d_asset = sqrt((assetLoc(1) - pos(1)).^2 + (assetLoc(2) - pos(2)).^2);
                        if d_asset <= (uasObj.speed * dt_local); eventAsset = true; end
                    end
                    
                    z_terr = terrainProxy(pos(1), pos(2)); 
                    eventCrash = (pos(3) <= z_terr);
                    eventExit = tick_count > 10 && ((pos(1) <= 0) || (pos(1) >= obj.map.size.horiz) || (pos(2) <= 0) || (pos(2) >= obj.map.size.vert));
                    
                    if eventEffector
                        if hasAsset
                            d_kill = norm(pos(1:2) - assetLoc(1:2));
                        else
                            d_kill = 50;
                        end
                        
                        risk_penalty = risk_max * exp(-risk_decay * d_kill);
                        
                        cost = cost + cost_eff + risk_penalty; 
                        outcomeLog(end+1) = "Intercept";
                        UASkillLocations = [UASkillLocations; pos];
                        uasObj.active = false; uas_active(i) = false; obj.UAS(i) = uasObj;
                        obj.directEnergyTrackTime(i) = 0;
                        UASkilled = UASkilled + 1;
                        if animate_on; obj.map.animateUASkilled(pos); end
                        
                    elseif eventCrash
                        cost = cost + cost_leak; outcomeLog(end+1) = "TerrainCrash";
                        uasObj.active = false; uas_active(i) = false; obj.UAS(i) = uasObj;
                        if animate_on; obj.map.animateUAScrashed(pos); end
                        
                    elseif eventExit
                        cost = cost + cost_leak; outcomeLog(end+1) = "Escaped";
                        uasObj.active = false; uas_active(i) = false; obj.UAS(i) = uasObj;
                        
                    elseif eventAsset
                        if ~isAssetDestroyed
                            isAssetDestroyed = true;
                            cost = cost + cost_asset; outcomeLog(end+1) = "AssetHit";
                            if animate_on; obj.map.animateDestroyedAsset(obj.asset); end
                        end
                    end
                end
                
                if hasEffectors && any(kineticMask)
                    if obj.kineticUseFermiModel
                        obj.launchKineticFermiShots_(currentTime, uas_active, uasTracked);
                        [uas_active, killedIdx, killedPos, missedPos] = obj.stepPendingKineticHits_(currentTime, uas_active);
                    else
                        obj.launchKineticProjectiles_(currentTime, uas_active, uasTracked);
                        [uas_active, killedIdx, killedPos, missedPos] = obj.stepProjectiles_(dt_local, uas_active);
                        if animate_on
                            obj.updateProjectileGraphics_();
                        end
                    end
                    
                    if animate_on
                        for mm = 1:size(missedPos, 1)
                            obj.map.animateUASmissed(missedPos(mm, :));
                        end
                    end

                    for kk = 1:numel(killedIdx)
                        iKill = killedIdx(kk);
                        uasObj = obj.UAS(iKill);
                        if uasObj.active 
                            uasObj.active = false;
                            obj.UAS(iKill) = uasObj;
                            obj.directEnergyTrackTime(iKill) = 0;
                            UASkilled = UASkilled + 1;
                            if hasAsset
                                d_kill = norm(killedPos(kk, 1:2) - assetLoc(1:2));
                            else
                                d_kill = mean([obj.map.size.horiz, obj.map.size.vert]);
                            end
                            risk_penalty = risk_max * exp(-risk_decay * d_kill);
                            
                            cost = cost + cost_eff + risk_penalty;
                            UASkillLocations = [UASkillLocations; killedPos(kk, :)]; 
                            if obj.kineticUseFermiModel
                                outcomeLog(end + 1) = "Intercept_Kinetic_Fermi";
                            else
                                outcomeLog(end + 1) = "Intercept_Kinetic";
                            end
                            if animate_on; obj.map.animateUASkilled(killedPos(kk, :)); end
                        end
                    end
                end
                
                if animate_on
                    pause(dt_local/obj.animationMultiplier);
                    obj.map.updateUASAnimation(obj.UASPos_all, obj.UASTrackedPos_all, uasTracked); % <--- PASS NEW ARRAY
                    if hasEffectors; obj.map.updateEffectors(obj.effectors); end
                    if ~obj.hideClock; obj.map.updateClock(currentTime); end
                end
            end
            
            if animate_on
                for i = 1:numUAS
                    valid_rows = ~isnan(obj.UASPos_all{i}(:,1));
                    obj.UASPos_all{i} = obj.UASPos_all{i}(valid_rows,:);
                    obj.UASTrackedPos_all{i} = obj.UASTrackedPos_all{i}(valid_rows,:); 
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

    methods (Access = private)
        function stepEffectors_(obj, targetPosXY, targetObj)
            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            for e = 1:numel(obj.effectors)
                eff = obj.effectors(e);
                if eff.mode == "STATIC"
                    continue; 
                end

                step = eff.speed * obj.dt;
                if step <= 0
                    continue;
                end

                % 1. Predict where the UAS will be
                interceptPoseRaw = obj.predictIntercept_(targetPosXY, targetObj);
                
                % 2. Sanitize current and goal positions to ensure they are on the map
                startPose = [eff.location eff.heading];
                if ~all(isfinite(startPose))
                    startPose = [eff.location 0];
                end
                startPose = obj.sanitizeMobilePose_(startPose, []);
                goalPose = obj.sanitizeMobilePose_(interceptPoseRaw, startPose);

                % 3. FAST REACTIVE PURSUIT (Bypasses heavy plannerHybridAStar)
                [newLoc, newHeading] = obj.moveTowardGoalSafely_(eff.location, eff.heading, goalPose, step);

                % 4. Clamp to boundaries and save state
                eff.location(1) = min(max(newLoc(1), xMin), xMax);
                eff.location(2) = min(max(newLoc(2), yMin), yMax);
                eff.heading = newHeading;
                
                obj.effectors(e) = eff;
            end
        end

        function interceptPose = predictIntercept_(obj, targetPosXY, targetObj)
            lookahead = 2.0;
            vhat = [1 0];

            if nargin >= 3 && ~isempty(targetObj) && isprop(targetObj, "targetUnitVector")
                v = targetObj.targetUnitVector(1:2);
                if all(isfinite(v)) && norm(v) > 0
                    vhat = v / norm(v);
                end
            end

            spd = 0;
            if nargin >= 3 && ~isempty(targetObj) && isprop(targetObj, "speed")
                spd = max(0, targetObj.speed);
            end

            ip = targetPosXY + vhat * spd * lookahead;
            hdg = atan2(vhat(2), vhat(1));
            interceptPose = obj.clampPose_([ip, hdg]);
        end

        function poseOut = sanitizeMobilePose_(obj, poseIn, fallbackPose)
            poseOut = obj.clampPose_(poseIn);

            if nargin < 3 || isempty(fallbackPose)
                fallbackPose = poseOut;
            else
                fallbackPose = obj.clampPose_(fallbackPose);
            end

            if ~obj.isPoseValid_(poseOut)
                poseOut = obj.nearestFreePose_(poseOut, fallbackPose);
            end

            if ~isfinite(poseOut(3))
                poseOut(3) = fallbackPose(3);
            end
        end

        function pose = clampPose_(obj, pose)
            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            if ~all(isfinite(pose))
                pose = [xMin, yMin, 0];
            end

            pose(1) = min(max(pose(1), xMin), xMax);
            pose(2) = min(max(pose(2), yMin), yMax);
            pose(3) = atan2(sin(pose(3)), cos(pose(3))); 
        end

        function tf = isPoseValid_(obj, pose)
            tf = all(isfinite(pose));
            if ~tf
                return
            end

            x = pose(1);
            y = pose(2);
            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            if x < xMin || x > xMax || y < yMin || y > yMax
                tf = false;
                return
            end

            try
                tf = ~checkOccupancy(obj.occMap, [x y]); 
            catch
                tf = true;
            end
        end

        function poseOut = nearestFreePose_(obj, poseIn, fallbackPose)
            poseOut = obj.clampPose_(poseIn);

            if obj.isPoseValid_(poseOut)
                return
            end

            if nargin >= 3 && ~isempty(fallbackPose)
                fallbackPose = obj.clampPose_(fallbackPose);
                if obj.isPoseValid_(fallbackPose)
                    poseOut = fallbackPose;
                    return
                end
            end

            radii = [0.5 1 2 3 5 8 12 16];
            angles = linspace(0, 2 * pi, 24);

            for r = radii
                for a = angles
                    candidate = poseOut;
                    candidate(1) = poseOut(1) + r * cos(a);
                    candidate(2) = poseOut(2) + r * sin(a);
                    candidate = obj.clampPose_(candidate);
                    if obj.isPoseValid_(candidate)
                        poseOut = candidate; 
                        return
                    end
                end
            end

            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            xs = linspace(xMin, xMax, 30);
            ys = linspace(yMin, yMax, 30);
            bestPose = poseOut;
            bestDist = inf;

            for ix = 1:numel(xs)
                for iy = 1:numel(ys)
                    candidate = [xs(ix), ys(iy), poseOut(3)];
                    if obj.isPoseValid_(candidate)
                        d = norm(candidate(1:2) - poseOut(1:2));
                        if d < bestDist
                            bestDist = d;
                            bestPose = candidate;
                        end
                    end
                end
            end

            poseOut = bestPose;
        end

        function tf = pathIsUsable_(obj, eff, goalPose, goalTolerance)
            tf = ~isempty(eff.path) && eff.pathIdx <= size(eff.path, 1);
            if ~tf
                return
            end

            if any(~isfinite(goalPose))
                tf = false;
                return
            end

            finalPose = obj.clampPose_(eff.path(end, :));
            if ~obj.isPoseValid_(finalPose)
                tf = false;
                return
            end

            tf = norm(finalPose(1:2) - goalPose(1:2)) <= goalTolerance;
        end

        function [newLoc, newHeading] = moveTowardGoalSafely_(obj, currentLoc, currentHeading, goalPose, step)
            newLoc = currentLoc;
            newHeading = currentHeading;

            goal = goalPose(1:2);
            v = goal - currentLoc;
            nv = norm(v);
            if nv <= 1e-9
                return
            end

            dir = v / nv;
            moveDist = min(step, nv);

            % Step purely in 2D. Elevation (Z) is handled automatically by the terrain mesh.
            candidateLoc = currentLoc + moveDist * dir;
            candidatePose = obj.clampPose_([candidateLoc, atan2(dir(2), dir(1))]);
            candidatePose = obj.sanitizeMobilePose_(candidatePose, [currentLoc, currentHeading]);

            if obj.isPoseValid_(candidatePose)
                newLoc = candidatePose(1:2);
                newHeading = candidatePose(3);
            end
        end

        function tf = hasLineOfSight_(obj, p0, p1, clearance, terrainProxy)
            if nargin < 4 || isempty(clearance)
                clearance = 0;
            end
            if nargin < 5
                terrainProxy = obj.map.terrainProxy;
            end
            n = max(25, ceil(norm(p1(1:2) - p0(1:2)) / 0.75));
            dx = (p1(1) - p0(1)) / (n - 1);
            dy = (p1(2) - p0(2)) / (n - 1);
            dz = (p1(3) - p0(3)) / (n - 1);
            
            cx = p0(1); cy = p0(2); cz = p0(3);
            tf = true;
            for i = 1:n
                zTerr = terrainProxy(cx, cy);
                if cz < (zTerr + clearance)
                    tf = false;
                    return;
                end
                cx = cx + dx; cy = cy + dy; cz = cz + dz;
            end
        end

        function tf = segmentHitsTerrain_(obj, p0, p1, clearance, terrainProxy)
            if nargin < 4 || isempty(clearance)
                clearance = 0;
            end
            if nargin < 5
                terrainProxy = obj.map.terrainProxy;
            end
            n = max(10, ceil(norm(p1(1:2) - p0(1:2)) / 0.5));
            dx = (p1(1) - p0(1)) / (n - 1);
            dy = (p1(2) - p0(2)) / (n - 1);
            dz = (p1(3) - p0(3)) / (n - 1);
            
            cx = p0(1); cy = p0(2); cz = p0(3);
            tf = false;
            for i = 1:n
                zTerr = terrainProxy(cx, cy);
                if cz <= (zTerr + clearance)
                    tf = true;
                    return;
                end
                cx = cx + dx; cy = cy + dy; cz = cz + dz;
            end
        end

        function p3 = getGroundPoint3_(obj, x, y, zOffset)
            if nargin < 5
                zOffset = 0;
            end
            z = obj.map.getElevation(x, y);
            p3 = [x, y, z + zOffset];
        end

        function launchKineticFermiShots_(obj, currentTime, uasActive, uasTracked)
            if isempty(obj.effectors) || isempty(obj.UAS); return; end

            for e = 1:numel(obj.effectors)
                if obj.effectors(e).weaponMode ~= "KINETIC"
                    continue;
                end
                
                if (currentTime - obj.effectorLastFireTime(e)) < obj.kineticRefireTime
                    continue;
                end

                effLoc = obj.effectors(e).location(1:2);
                effRange = obj.effectors(e).range;

                if size(obj.effectors3D, 1) >= e && all(isfinite(obj.effectors3D(e, :)))
                    shooterPos3 = obj.effectors3D(e, :);
                else
                    shooterPos3 = obj.getGroundPoint3_(effLoc(1), effLoc(2), simulator.effectorHeight_());
                end

                candidateIdx = []; candidateDist = inf; candidateTargetPos3 = [];
                for i = 1:numel(obj.UAS)
                    if ~uasActive(i) || (nargin >= 4 && ~isempty(uasTracked) && ~uasTracked(i)); continue; end
                    
                    targetObj = obj.UAS(i);
                    targetPos3 = targetObj.position(1:3);
                    d = norm(targetPos3(1:2) - effLoc);

                    if d > effRange || ~obj.hasLineOfSight_(shooterPos3, targetPos3, simulator.losClearance_(), obj.map.terrainProxy)
                        continue
                    end

                    if d < candidateDist
                        candidateIdx = i; candidateDist = d; candidateTargetPos3 = targetPos3;
                    end
                end

                if isempty(candidateIdx); continue; end

                pShot = obj.fermiShotProbability_(candidateDist, effRange);
                pVolley = min(1, max(0, 1 - (1 - pShot)^obj.kineticShotsPerVolley));
                travelTime = norm(candidateTargetPos3 - shooterPos3) / max(obj.kineticProjectileSpeed, eps);
                
                pending = simulator.emptyPendingShot_();
                pending.active = true; pending.effectorIdx = e; pending.targetIdx = candidateIdx;
                pending.fireTime = currentTime; pending.hitTime = currentTime + travelTime;
                pending.willHit = (rand(obj.randomStream) <= pVolley);

                if obj.numPendingKineticHits < obj.maxPendingKineticHits
                    obj.numPendingKineticHits = obj.numPendingKineticHits + 1;
                    obj.pendingKineticHits(obj.numPendingKineticHits, 1) = pending;
                end
                obj.effectorLastFireTime(e) = currentTime;
            end
        end

        function pShot = fermiShotProbability_(obj, distanceToTarget, effectorRange)
            d50 = max(0.5, obj.kineticFermiD50Scale * effectorRange);
            k = max(0.10, obj.kineticFermiSharpnessScale * effectorRange);
            pShot = min(1, max(0, obj.kineticHitProbability / (1 + exp((distanceToTarget - d50) / k))));
        end

        function [uasActive, killedIdx, killedPos, missedPos] = stepPendingKineticHits_(obj, currentTime, uasActive)
            killedIdx = zeros(0, 1); killedPos = zeros(0, 3); missedPos = zeros(0, 3);
            if obj.numPendingKineticHits == 0; return; end

            keepCount = 0;
            for sIdx = 1:obj.numPendingKineticHits
                shot = obj.pendingKineticHits(sIdx);
                if ~shot.active; continue; end
                
                if currentTime < shot.hitTime
                    keepCount = keepCount + 1;
                    obj.pendingKineticHits(keepCount) = shot;
                    continue; 
                end

                tIdx = shot.targetIdx;
                if tIdx >= 1 && tIdx <= numel(obj.UAS) && uasActive(tIdx) 
                    if shot.willHit
                        uasActive(tIdx) = false;
                        killedIdx(end + 1, 1) = tIdx; 
                        killedPos(end + 1, :) = obj.UAS(tIdx).position; 
                    else
                        missedPos(end + 1, :) = obj.UAS(tIdx).position; 
                    end
                end
            end
            obj.numPendingKineticHits = keepCount;
        end

        function launchKineticProjectiles_(obj, currentTime, uasActive, uasTracked) 
            if isempty(obj.effectors) || isempty(obj.UAS); return; end

            for e = 1:numel(obj.effectors)
                if obj.effectors(e).weaponMode ~= "KINETIC"
                    continue;
                end
                
                if (currentTime - obj.effectorLastFireTime(e)) < obj.kineticRefireTime
                    continue;
                end

                effLoc = obj.effectors(e).location(1:2);
                effRange = obj.effectors(e).range;

                if size(obj.effectors3D, 1) >= e && all(isfinite(obj.effectors3D(e, :)))
                    shooterPos3 = obj.effectors3D(e, :);
                else
                    shooterPos3 = obj.getGroundPoint3_(effLoc(1), effLoc(2), simulator.effectorHeight_());
                end

                candidateIdx = []; candidateDist = inf; candidateAimPoint = [];
                for i = 1:numel(obj.UAS)
                    if ~uasActive(i) || (nargin >= 4 && ~isempty(uasTracked) && ~uasTracked(i)); continue; end

                    uasPos = obj.UAS(i).position(1:2);
                    d = norm(uasPos - effLoc);

                    targetObj = obj.UAS(i);
                    targetPos3 = targetObj.position(1:3);

                    if d > effRange || ~obj.hasLineOfSight_(shooterPos3, targetPos3, simulator.losClearance_(), obj.map.terrainProxy)
                        continue
                    end

                    [aimPoint3, canSolve] = obj.solveLinearIntercept_(shooterPos3, targetPos3, targetObj);
                    if ~canSolve; aimPoint3 = targetPos3; end

                    if d < candidateDist
                        candidateIdx = i; candidateDist = d; candidateAimPoint = aimPoint3;
                    end
                end

                if isempty(candidateIdx); continue; end

                dir3 = candidateAimPoint - shooterPos3;
                nDir = norm(dir3);
                if nDir < eps; continue; end
                vel3 = obj.kineticProjectileSpeed * (dir3 / nDir);

                for s = 1:obj.kineticShotsPerVolley
                    p = simulator.emptyProjectile_();
                    p.active = true; p.pos = shooterPos3; p.prevPos = shooterPos3; p.vel = vel3;
                    p.targetIdx = candidateIdx; p.hitEligible = (rand(obj.randomStream) <= obj.kineticHitProbability);
                    p.maxLife = max(1.0, 2.0 * max(norm(candidateAimPoint - shooterPos3), eps) / obj.kineticProjectileSpeed);
                    p.trail = shooterPos3;

                    if ~p.hitEligible
                        missDir = randn(obj.randomStream, 1, 3);
                        missDir = missDir / max(norm(missDir), 1e-6);
                        p.missOffset = missDir * max(1.5, obj.projectileHitTolerance * 2);
                    end
                    
                    if obj.numProjectiles < obj.maxProjectiles
                        obj.numProjectiles = obj.numProjectiles + 1;
                        obj.projectiles(obj.numProjectiles, 1) = p;
                    end
                end
                obj.effectorLastFireTime(e) = currentTime;
            end
        end

        function [uasActive, killedIdx, killedPos, missedPos] = stepProjectiles_(obj, dtLocal, uasActive)
            killedIdx = zeros(0, 1); killedPos = zeros(0, 3); missedPos = zeros(0, 3);
            if obj.numProjectiles == 0; return; end

            keepCount = 0;
            xMin = obj.occMap.XWorldLimits(1); xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1); yMax = obj.occMap.YWorldLimits(2);

            for pIdx = 1:obj.numProjectiles
                p = obj.projectiles(pIdx);
                if ~p.active; continue; end

                oldPos = p.pos;
                p.prevPos = oldPos; p.pos = p.pos + p.vel * dtLocal;
                p.life = p.life + dtLocal; p.trail = [p.trail; p.pos]; 

                if p.pos(1) < xMin || p.pos(1) > xMax || p.pos(2) < yMin || p.pos(2) > yMax || ...
                   p.life > p.maxLife || obj.segmentHitsTerrain_(oldPos, p.pos, 0.02, obj.map.terrainProxy)
                    continue; 
                end

                tIdx = p.targetIdx;
                if tIdx >= 1 && tIdx <= numel(obj.UAS) && uasActive(tIdx)
                    targetPos = obj.UAS(tIdx).position(1:3);
                    if ~p.hitEligible; targetPos = targetPos + p.missOffset; end

                    if simulator.pointToSegmentDistance3D_(targetPos, oldPos, p.pos) <= obj.projectileHitTolerance
                        if p.hitEligible
                            uasActive(tIdx) = false;
                            killedIdx(end + 1, 1) = tIdx; 
                            killedPos(end + 1, :) = obj.UAS(tIdx).position; 
                        else
                            missedPos(end + 1, :) = obj.UAS(tIdx).position; 
                        end
                        continue;
                    end
                end
                
                keepCount = keepCount + 1;
                obj.projectiles(keepCount) = p;
            end
            obj.numProjectiles = keepCount;
        end

        function [aimPoint, canSolve] = solveLinearIntercept_(obj, shooterPos, targetPos, uasObj)
            canSolve = false; aimPoint = targetPos;
            vhat = [1 0];
            if isprop(uasObj, "targetUnitVector")
                v = uasObj.targetUnitVector(1:2);
                if all(isfinite(v)) && norm(v) > 0; vhat = v / norm(v); end
            end
            vTarget = [0 0 0];
            if isprop(uasObj, "speed"); vTarget = [max(0, uasObj.speed) * vhat, 0]; end

            r = targetPos - shooterPos;
            s = obj.kineticProjectileSpeed;
            a = dot(vTarget, vTarget) - s^2;
            b = 2 * dot(r, vTarget);
            c = dot(r, r);

            if abs(a) < 1e-12
                t = (abs(b) < 1e-12) * 0 + (abs(b) >= 1e-12) * (-c / b);
                if t > 0; aimPoint = targetPos + vTarget * t; canSolve = true; end
                return
            end

            disc = b^2 - 4 * a * c;
            if disc < 0; return; end

            tCandidates = [(-b + sqrt(disc)) / (2 * a), (-b - sqrt(disc)) / (2 * a)];
            tCandidates = tCandidates(tCandidates > 0);
            if ~isempty(tCandidates)
                aimPoint = targetPos + vTarget * min(tCandidates);
                canSolve = true;
            end
        end

        function updateProjectileGraphics_(obj)
            if ~isempty(obj.projectileHandles)
                for k = 1:numel(obj.projectileHandles)
                    if isgraphics(obj.projectileHandles(k)); delete(obj.projectileHandles(k)); end
                    if isgraphics(obj.projectileTrailHandles(k)); delete(obj.projectileTrailHandles(k)); end
                end
            end
            if obj.numProjectiles == 0; return; end
            
            ax = gca; holdState = ishold(ax); hold(ax, 'on');
            nProj = obj.numProjectiles;
            obj.projectileHandles = gobjects(nProj, 1);
            obj.projectileTrailHandles = gobjects(nProj, 1);

            for k = 1:nProj
                pos = obj.projectiles(k).pos; trail = obj.projectiles(k).trail;
                obj.projectileTrailHandles(k) = plot3(ax, trail(:, 1), trail(:, 2), trail(:, 3), 'r-', 'LineWidth', 2);
                obj.projectileHandles(k) = plot3(ax, pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
            if ~holdState; hold(ax, 'off'); end
        end
    end

    methods (Static, Access = private)
        function p = emptyProjectile_()
            p = struct('active', false, 'pos', [0 0 0], 'prevPos', [0 0 0], 'vel', [0 0 0], ...
                'targetIdx', 0, 'hitEligible', false, 'missOffset', [0 0 0], 'life', 0, 'maxLife', 0, 'trail', zeros(0, 3));
        end

        function s = emptyPendingShot_()
            s = struct('active', false, 'effectorIdx', 0, 'targetIdx', 0, 'fireTime', 0, 'hitTime', 0, ...
                'pShot', 0, 'pVolley', 0, 'travelTime', 0, 'rangeAtFire', 0, 'willHit', false);
        end

        function d = pointToSegmentDistance3D_(pt, a, b)
            ab = b - a; denom = dot(ab, ab);
            if denom <= eps; d = norm(pt - a); return; end
            t = max(0, min(1, dot(pt - a, ab) / denom));
            d = norm(pt - (a + t * ab));
        end

        function h = effectorHeight_()
            h = 1.2;
        end

        function c = losClearance_()
            c = 0.10;
        end
    end
end