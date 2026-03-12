classdef simulator < handle
    properties
        map
        AOR
        UAS
        UASPos_all
        effectors
        sensors
        assets

        defenders
        obstacles
        NFZunion

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

        grid2D
        showFigure
        maxSimTime
        drawEvery

        stopRule
        defenderInterceptMode
        defenderInterceptRadius
        defenderCenterHitTol

        effectorHitMode
        effectorCenterHitTol

        weaponMode
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

        terrainHandle
    end

    methods
        function obj = simulator(map, aor, uas, effectors, sensors, assets, options)
            arguments
                map, aor, uas, effectors, sensors, assets
                options.tps = 20
                options.animate = true
                options.nfzs = polyshape.empty
                options.resetGraphics = true
                options.animationMultiplier = 1
                options.hideClock = false
                options.fadePings = false
                options.costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250)
                options.defenders = []
                options.obstacles = polyshape.empty
                options.grid2D (1, 1) logical = true

                options.showFigure (1, 1) logical = false
                options.maxSimTime (1, 1) double = inf
                options.drawEvery (1, 1) double = 1

                options.stopRule (1, 1) string = "ALL_INTERCEPT_OR_ANY_ASSET"

                options.defenderInterceptMode (1, 1) string = "RADIUS"
                options.defenderInterceptRadius (1, 1) double = NaN
                options.defenderCenterHitTol (1, 1) double = 0.35

                options.effectorHitMode (1, 1) string = "RING"
                options.effectorCenterHitTol (1, 1) double = 0.35

                options.weaponMode (1, 1) string = "LEGACY"
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

            obj.map = map;
            obj.AOR = aor;
            obj.UAS = uas;
            obj.effectors = effectors;
            obj.sensors = sensors;
            obj.assets = assets;

            obj.tick = 0;
            obj.tps = options.tps;
            obj.dt = 1 / obj.tps;

            obj.animate = options.animate;
            obj.NFZs = options.nfzs;
            obj.resetGraphics = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier;
            obj.hideClock = options.hideClock;
            obj.fadePings = options.fadePings;
            obj.costConfig = options.costConfig;

            obj.defenders = options.defenders;
            obj.obstacles = options.obstacles;

            obj.grid2D = options.grid2D;
            obj.showFigure = options.showFigure;
            obj.maxSimTime = options.maxSimTime;
            obj.drawEvery = max(1, round(options.drawEvery));

            obj.stopRule = upper(string(options.stopRule));

            obj.defenderInterceptMode = upper(string(options.defenderInterceptMode));
            obj.defenderInterceptRadius = options.defenderInterceptRadius;
            obj.defenderCenterHitTol = options.defenderCenterHitTol;

            obj.effectorHitMode = upper(string(options.effectorHitMode));
            obj.effectorCenterHitTol = options.effectorCenterHitTol;

            obj.weaponMode = upper(string(options.weaponMode));
            if obj.weaponMode == "KINECT"
                obj.weaponMode = "KINETIC";
            end

            obj.directEnergyDwellTime = max(0, options.directEnergyDwellTime);
            obj.kineticProjectileSpeed = max(eps, options.kineticProjectileSpeed);
            obj.kineticShotsPerVolley = max(1, round(options.kineticShotsPerVolley));
            obj.kineticHitProbability = min(1, max(0, options.kineticHitProbability));
            obj.kineticRefireTime = max(0, options.kineticRefireTime);
            obj.projectileHitTolerance = max(0.01, options.projectileHitTolerance);

            obj.kineticFermiD50Scale = max(0.01, options.kineticFermiD50Scale);
            obj.kineticFermiSharpnessScale = max(0.01, options.kineticFermiSharpnessScale);
            obj.kineticUseFermiModel = options.kineticUseFermiModel;

            obj.NFZunion = simulator.unionPolys(obj.NFZs);

            obj.occMap = obj.map.getOccMap(28);

            obj.UASPos_all = cell(1, length(obj.UAS));
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
            end

            obj.effectors3D = simulator.buildEffectors3D_(obj.map, obj.effectors);

            obj.directEnergyTrackTime = zeros(length(obj.UAS), 1);
            obj.effectorLastFireTime = -inf(max(numel(obj.effectors), 1), 1);
            obj.projectiles = repmat(simulator.emptyProjectile_(), 0, 1);
            obj.projectileHandles = gobjects(0);
            obj.projectileTrailHandles = gobjects(0);
            obj.pendingKineticHits = repmat(simulator.emptyPendingShot_(), 0, 1);

            obj.terrainHandle = gobjects(0);
        end

        function results = runSim(obj)
            dtLocal = obj.dt;
            costEff = obj.costConfig.effector;
            costLeak = obj.costConfig.leak;
            costAsset = obj.costConfig.asset;

            hasSensors = ~isempty(obj.sensors);

            useSensorParamsModel = false;
            if hasSensors && isstruct(obj.sensors) && isfield(obj.sensors(1), "params")
                p = [obj.sensors.params];
                if all(isfield(p, "d50")) && all(isfield(p, "k")) && isfield(p(1), "pings") && isfield(p(1), "scanRate")
                    useSensorParamsModel = true;
                end
            end

            if hasSensors && useSensorParamsModel
                p = [obj.sensors.params];
                sensorD50 = [p.d50];
                sensorK = [p.k];
                reqPings = p(1).pings;
                scanRate = p(1).scanRate;
                sensorLocs = reshape([obj.sensors.location], 2, [])';
            else
                reqPings = 1;
                scanRate = 1;
                sensorLocs = [];
                sensorD50 = [];
                sensorK = [];
            end

            hasEffectors = ~isempty(obj.effectors);

            hasAssets = ~isempty(obj.assets);
            if hasAssets
                assetLocs = reshape([obj.assets.location], 2, [])';
            else
                assetLocs = [];
            end

            terrainProxy = obj.map.terrainProxy;
            numUAS = length(obj.UAS);
            uasActive = true(numUAS, 1);

            trackHist = zeros(numUAS, reqPings);

            destroyedAssets = [];
            cost = 0;
            UASkilled = 0;
            UASkillLocations = [];
            outcomeLog = strings(0);

            NFZEntered = false;
            UASsensedPos = [];

            nIntercept = 0;
            nAssetHit = 0;
            nCrash = 0;
            nNFZ = 0;

            msgAssetShown = false;
            msgInterceptShown = false;

            showFig = obj.animate || obj.showFigure;
            animateOn = obj.animate;

            defenderInterceptRadius = 0.75;
            if isfinite(obj.defenderInterceptRadius) && obj.defenderInterceptRadius > 0
                defenderInterceptRadius = obj.defenderInterceptRadius;
            end
            defenderCenterHitTol = max(0.01, obj.defenderCenterHitTol);

            effectorCenterHitTol = max(0.01, obj.effectorCenterHitTol);

            if showFig
                obj.map.startAnimation(obj.AOR, obj.assets, obj.effectors, obj.sensors, numUAS, obj.hideClock, ...
                    'NFZ', obj.NFZunion, ...
                    'Obstacles', obj.obstacles, ...
                    'Defenders', obj.defenders, ...
                    'Use2DRings', true, ...
                    'Grid2D', obj.grid2D);

                try
                    obj.renderTerrain3D_();
                    obj.map.updateUASAnimation(obj.UASPos_all);
                    obj.map.updateDefenders(obj.defenders);
                    if hasEffectors && ismethod(obj.map, "updateEffectors")
                        obj.map.updateEffectors(obj.effectors);
                    end

                    if obj.grid2D
                        if ~obj.hideClock
                            obj.map.updateClock(0);
                        end
                        obj.map.setEventMessage("Status: running...");
                    else
                        obj.update3DTitle_(0, 0, numUAS, 0, 0, 0);
                    end
                catch
                end
                drawnow;
            end

            simComplete = false;
            tickCount = 0;
            abortedByUser = false;

            stopNow = false;
            stopReason = "";
            currentTime = 0;

            try
                while ~simComplete
                    simComplete = true;
                    tickCount = tickCount + 1;
                    currentTime = tickCount * dtLocal;

                    if currentTime >= obj.maxSimTime
                        break
                    end

                    obj.tick = tickCount;

                    if showFig
                        if obj.grid2D && ~obj.hideClock
                            try
                                obj.map.updateClock(currentTime);
                            catch
                            end
                        elseif ~obj.grid2D
                            try
                                obj.update3DTitle_(currentTime, nIntercept, sum(uasActive), nAssetHit, nCrash, nNFZ);
                            catch
                            end
                        end
                    end

                    timeSinceLastScan = mod(currentTime, scanRate);
                    isScanTick = (timeSinceLastScan < dtLocal / 2) || (abs(timeSinceLastScan - scanRate) < dtLocal / 2);

                    targetFound = false;
                    targetPos = [];
                    targetObj = [];

                    uasTracked = false(numUAS, 1);

                    for i = 1:numUAS
                        if ~uasActive(i)
                            continue
                        end
                        simComplete = false;

                        uasObj = obj.UAS(i);
                        modeStr = upper(string(uasObj.mode));

                        switch modeStr
                            case "LINEAR"
                                uasObj.linearMotion(dtLocal);
                            case "HYBRIDASTAR"
                                uasObj.hybridAStarMotion(dtLocal, tickCount, 3.0, obj.occMap);
                            case "SEARCH"
                                uasObj.searchMotion(dtLocal, obj.assets, destroyedAssets, obj.NFZs);
                            otherwise
                        end

                        obj.UAS(i) = uasObj;
                        pos = uasObj.position;

                        if ~targetFound
                            targetFound = true;
                            targetPos = pos(1:2);
                            targetObj = uasObj;
                        end

                        if showFig
                            obj.UASPos_all{i} = cat(1, obj.UASPos_all{i}, pos);
                        end

                        isTracked = false;
                        isPinged = false;

                        if hasSensors && isScanTick && useSensorParamsModel
                            dSens = sqrt((sensorLocs(:, 1) - pos(1)).^2 + (sensorLocs(:, 2) - pos(2)).^2);
                            rawProbs = 1 ./ (1 + exp((dSens - sensorD50') ./ sensorK'));
                            probs = min(rawProbs, 0.90);
                            if any(probs >= rand(size(probs)))
                                isPinged = true;
                            end
                            trackHist(i, :) = [trackHist(i, 2:end), isPinged];
                        end

                        if sum(trackHist(i, :)) >= reqPings
                            isTracked = true;
                            uasTracked(i) = true;
                        end

                        if isPinged && showFig
                            UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                            try
                                obj.map.animateUASsensed(UASsensedPos);
                            catch
                            end
                        end

                        eventEffector = false;

                        if hasEffectors && isTracked && ~isempty(obj.effectors3D)
                            switch obj.weaponMode
                                case "LEGACY"
                                    effLocs = obj.effectors3D;
                                    effRanges = [obj.effectors.range]';
                                    dEff = sqrt(sum((effLocs(:, 1:2) - pos(1:2)).^2, 2));

                                    losOK = false(size(dEff));
                                    for ee = 1:numel(dEff)
                                        if all(isfinite(effLocs(ee, :)))
                                            losOK(ee) = obj.hasLineOfSight_(effLocs(ee, :), pos(1:3), simulator.losClearance_());
                                        end
                                    end

                                    hitByRing = any((dEff <= effRanges) & losOK);
                                    hitByCenter = any((dEff <= effectorCenterHitTol) & losOK);

                                    switch obj.effectorHitMode
                                        case "RING"
                                            eventEffector = hitByRing;
                                        case "CENTER"
                                            eventEffector = hitByCenter;
                                        otherwise
                                            eventEffector = hitByRing || hitByCenter;
                                    end

                                case "DIRECT_ENERGY"
                                    dEff = sqrt(sum((obj.effectors3D(:, 1:2) - pos(1:2)).^2, 2));
                                    inRangeMask = (dEff <= [obj.effectors.range]');
                                    losMask = false(size(inRangeMask));

                                    for ee = 1:numel(inRangeMask)
                                        if inRangeMask(ee) && all(isfinite(obj.effectors3D(ee, :)))
                                            losMask(ee) = obj.hasLineOfSight_(obj.effectors3D(ee, :), pos(1:3), simulator.losClearance_());
                                        end
                                    end

                                    inRange = any(inRangeMask & losMask);
                                    if inRange
                                        obj.directEnergyTrackTime(i) = obj.directEnergyTrackTime(i) + dtLocal;
                                    else
                                        obj.directEnergyTrackTime(i) = 0;
                                    end

                                    if obj.directEnergyTrackTime(i) >= obj.directEnergyDwellTime
                                        eventEffector = true;
                                    end

                                case "KINETIC"
                            end
                        else
                            if obj.weaponMode == "DIRECT_ENERGY"
                                obj.directEnergyTrackTime(i) = 0;
                            end
                        end

                        eventAsset = false;
                        hitAssetID = 0;
                        if hasAssets
                            dAsset = sqrt((assetLocs(:, 1) - pos(1)).^2 + (assetLocs(:, 2) - pos(2)).^2);
                            hitIdx = find(dAsset <= (uasObj.speed * dtLocal));
                            if ~isempty(hitIdx)
                                eventAsset = true;
                                hitAssetID = hitIdx(1);
                            end
                        end

                        zTerr = terrainProxy(pos(2), pos(1));
                        eventCrash = (pos(3) <= zTerr);

                        eventNFZ = false;
                        if ~isempty(obj.NFZunion)
                            eventNFZ = isinterior(obj.NFZunion, pos(1), pos(2));
                        end

                        if eventEffector
                            cost = cost + costEff;
                            outcomeLog(end + 1) = "Intercept";
                            UASkillLocations = [UASkillLocations; pos]; 

                            uasObj.active = false;
                            uasActive(i) = false;
                            obj.UAS(i) = uasObj;
                            obj.directEnergyTrackTime(i) = 0;

                            UASkilled = UASkilled + 1;
                            nIntercept = nIntercept + 1;

                            if ~msgInterceptShown
                                msgInterceptShown = true;
                                fprintf("t=%.2f: ADVERSARY INTERCEPTED.\n", currentTime);
                            end
                            if showFig
                                try
                                    obj.map.animateUASkilled(pos);
                                catch
                                end
                            end

                        elseif eventCrash
                            cost = cost + costLeak;
                            outcomeLog(end + 1) = "TerrainCrash";
                            uasObj.active = false;
                            uasActive(i) = false;
                            obj.UAS(i) = uasObj;
                            obj.directEnergyTrackTime(i) = 0;
                            nCrash = nCrash + 1;

                            if showFig
                                try
                                    obj.map.animateUAScrashed(pos);
                                catch
                                end
                            end

                        elseif eventNFZ
                            cost = cost + costLeak;
                            outcomeLog(end + 1) = "NFZEntered";
                            NFZEntered = true;
                            uasObj.active = false;
                            uasActive(i) = false;
                            obj.UAS(i) = uasObj;
                            obj.directEnergyTrackTime(i) = 0;
                            nNFZ = nNFZ + 1;

                        elseif eventAsset
                            if ~any(destroyedAssets == hitAssetID)
                                destroyedAssets(end + 1) = hitAssetID; 
                                cost = cost + costAsset;
                                outcomeLog(end + 1) = "AssetHit";
                                nAssetHit = nAssetHit + 1;

                                if ~msgAssetShown
                                    msgAssetShown = true;
                                    fprintf("t=%.2f: ADVERSARY HIT ASSET #%d.\n", currentTime, hitAssetID);
                                end
                                if showFig
                                    try
                                        obj.map.animateDestroyedAssets(obj.assets, destroyedAssets);
                                    catch
                                    end
                                end
                            end

                            stopNow = true;
                            stopReason = sprintf("Asset Hit (#%d)", hitAssetID);
                            break
                        end
                    end

                    if stopNow
                        simComplete = true;
                    end

                    if ~stopNow && ~isempty(obj.defenders) && targetFound
                        obj.stepDefenders_(targetPos, targetObj);

                        if hasEffectors
                            obj.effectors = obj.attachEffectorsToDefenders_(obj.effectors, obj.defenders);
                            obj.effectors3D = simulator.buildEffectors3D_(obj.map, obj.effectors, true);
                        end
                    end

                    if ~stopNow && obj.weaponMode == "KINETIC" && hasEffectors
                        if obj.kineticUseFermiModel
                            obj.launchKineticFermiShots_(currentTime, uasActive, uasTracked);
                        else
                            obj.launchKineticProjectiles_(currentTime, uasActive, uasTracked);

                            if showFig
                                try
                                    obj.updateProjectileGraphics_();
                                catch
                                end
                                drawnow limitrate;
                            end
                        end
                    end

                    if ~stopNow && obj.weaponMode == "KINETIC"
                        if obj.kineticUseFermiModel
                            [uasActive, killedIdx, killedPos] = obj.stepPendingKineticHits_(currentTime, uasActive);
                        else
                            [uasActive, killedIdx, killedPos] = obj.stepProjectiles_(dtLocal, uasActive);
                        end

                        for kk = 1:numel(killedIdx)
                            iKill = killedIdx(kk);
                            if iKill < 1 || iKill > numUAS
                                continue
                            end

                            uasObj = obj.UAS(iKill);
                            if uasObj.active
                                uasObj.active = false;
                                obj.UAS(iKill) = uasObj;
                            end

                            obj.directEnergyTrackTime(iKill) = 0;
                            UASkilled = UASkilled + 1;
                            nIntercept = nIntercept + 1;
                            cost = cost + costEff;
                            UASkillLocations = [UASkillLocations; killedPos(kk, :)]; 

                            if obj.kineticUseFermiModel
                                outcomeLog(end + 1) = "Intercept_Kinetic_Fermi";
                                fprintf("t=%.2f: KINETIC FERMI HIT on UAS %d.\n", currentTime, iKill);
                            else
                                outcomeLog(end + 1) = "Intercept_Kinetic";
                                fprintf("t=%.2f: KINETIC HIT on UAS %d.\n", currentTime, iKill);
                            end

                            if showFig
                                try
                                    obj.map.animateUASkilled(killedPos(kk, :));
                                catch
                                end
                            end
                        end
                    end

                    if ~stopNow && ~isempty(obj.defenders)
                        for i = 1:numUAS
                            if ~uasActive(i)
                                continue
                            end
                            pos = obj.UAS(i).position;
                            uasXY = pos(1:2);

                            dmin = inf;
                            for d = 1:numel(obj.defenders)
                                defLoc = obj.defenders(d).location(1:2);
                                dmin = min(dmin, norm(defLoc - uasXY));
                            end

                            if obj.defenderInterceptMode == "CENTER"
                                isContact = (dmin <= defenderCenterHitTol);
                            else
                                isContact = (dmin <= defenderInterceptRadius);
                            end

                            if isContact
                                uasObj = obj.UAS(i);
                                uasObj.active = false;
                                obj.UAS(i) = uasObj;
                                uasActive(i) = false;
                                obj.directEnergyTrackTime(i) = 0;

                                UASkilled = UASkilled + 1;
                                nIntercept = nIntercept + 1;

                                UASkillLocations = [UASkillLocations; pos]; 
                                outcomeLog(end + 1) = "Intercept_DefenderContact";

                                if showFig
                                    try
                                        obj.map.animateUASkilled(pos);
                                    catch
                                    end
                                end
                            end
                        end
                    end

                    if ~stopNow && obj.stopRule == "ALL_INTERCEPT_OR_ANY_ASSET"
                        if UASkilled >= numUAS
                            stopNow = true;
                            stopReason = "All adversaries intercepted";
                            simComplete = true;
                        end
                    end

                    if showFig
                        try
                            nActive = sum(uasActive);
                            if obj.grid2D
                                obj.map.setEventMessage(sprintf("Intercepted=%d | Active=%d | AssetHit=%d | Crash=%d | NFZ=%d", nIntercept, nActive, nAssetHit, nCrash, nNFZ));
                            else
                                obj.update3DTitle_(currentTime, nIntercept, nActive, nAssetHit, nCrash, nNFZ);
                            end
                        catch
                        end
                    end

                    if showFig && mod(tickCount, obj.drawEvery) == 0
                        try
                            obj.renderTerrain3D_();
                        catch MEgfx
                            fprintf(2, "DRAW WARNING TERRAIN (tick %d): %s\n", tickCount, MEgfx.message);
                        end

                        try
                            obj.map.updateUASAnimation(obj.UASPos_all);
                        catch MEgfx
                            fprintf(2, "DRAW WARNING UAS (tick %d): %s\n", tickCount, MEgfx.message);
                        end

                        try
                            obj.map.updateDefenders(obj.defenders);
                        catch MEgfx
                            fprintf(2, "DRAW WARNING DEFENDERS (tick %d): %s\n", tickCount, MEgfx.message);
                        end

                        if hasEffectors && ismethod(obj.map, "updateEffectors")
                            try
                                obj.map.updateEffectors(obj.effectors);
                            catch MEgfx
                                fprintf(2, "DRAW WARNING EFFECTORS (tick %d): %s\n", tickCount, MEgfx.message);
                            end
                        end

                        if ~obj.kineticUseFermiModel
                            try
                                obj.updateProjectileGraphics_();
                            catch MEgfx
                                fprintf(2, "DRAW WARNING PROJECTILES (tick %d): %s\n", tickCount, MEgfx.message);
                            end
                        end

                        if animateOn
                            pause(dtLocal / max(obj.animationMultiplier, eps));
                        end
                        drawnow limitrate;
                    end
                end

            catch ME
                fprintf(2, "\nSIM ERROR: %s\n", ME.message);
                for si = 1:numel(ME.stack)
                    fprintf(2, "  at %s (line %d)\n", ME.stack(si).name, ME.stack(si).line);
                end
                fprintf(2, "\n");
                abortedByUser = true;
            end

            if showFig
                try
                    if obj.grid2D
                        if stopNow
                            obj.map.setEventMessage(sprintf("STOP: %s | Intercepted=%d/%d | AssetHit=%d", stopReason, UASkilled, numUAS, nAssetHit));
                        else
                            obj.map.setEventMessage(sprintf("FINAL: Intercepted=%d/%d | AssetHit=%d | Crash=%d | NFZ=%d", UASkilled, numUAS, nAssetHit, nCrash, nNFZ));
                        end
                    else
                        obj.update3DTitle_(currentTime, UASkilled, sum(uasActive), nAssetHit, nCrash, nNFZ);
                    end
                catch
                end
            end

            results.UASPos_all = obj.UASPos_all;
            results.destroyedAssets = destroyedAssets;
            results.cost = cost;
            results.UASkilled = UASkilled;
            results.UASkillLocations = UASkillLocations;
            results.outcomeLog = outcomeLog;
            results.tick = tickCount;
            results.NFZEntered = NFZEntered;
            results.UASsensedPos = UASsensedPos;
            results.abortedByUser = abortedByUser;

            results.nIntercept = nIntercept;
            results.nAssetHit = nAssetHit;
            results.nCrash = nCrash;
            results.nNFZ = nNFZ;

            results.stopNow = stopNow;
            results.stopReason = stopReason;

            results.effectorHitMode = obj.effectorHitMode;
            results.effectorCenterHitTol = effectorCenterHitTol;
            results.weaponMode = obj.weaponMode;
            results.kineticUseFermiModel = obj.kineticUseFermiModel;
        end
    end

    methods (Access = private)
        function stepDefenders_(obj, uasPosXY, uasObj)
            if isempty(obj.defenders)
                return
            end
            if isstruct(obj.defenders)
                obj.defenders = simulator.normalizeDefenderStructArray_(obj.defenders);
            end

            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            for d = 1:numel(obj.defenders)
                def = obj.defenders(d);

                mode = "HYBRIDA*";
                if isfield(def, "mode")
                    mode = upper(string(def.mode));
                end
                if mode == "MOBILE"
                    mode = "HYBRIDA*";
                end
                if mode == "STATIC"
                    obj.defenders(d) = def;
                    continue
                end

                speed = 10;
                if isfield(def, "speed")
                    speed = def.speed;
                end
                if isempty(speed) || speed <= 0
                    speed = 10;
                end
                step = speed * obj.dt;

                replanEveryTicks = 30;
                replanDist = 5.0;

                interceptPose = obj.predictIntercept_(def, uasPosXY, uasObj);

                try
                    setOccupancy(obj.occMap, def.location(1:2), 0, "world");
                    setOccupancy(obj.occMap, interceptPose(1:2), 0, "world");
                catch
                end

                if isempty(def.planner)
                    ss = stateSpaceSE2;
                    ss.StateBounds = [obj.occMap.XWorldLimits; obj.occMap.YWorldLimits; -pi pi];

                    sv = validatorOccupancyMap(ss);
                    sv.Map = obj.occMap;
                    sv.ValidationDistance = 0.5;

                    def.planner = plannerHybridAStar(sv, 'MinTurningRadius', 3.0, "InterpolationDistance", step);
                    def.path = [];
                    def.pathIdx = 1;
                    def.lastPlanTick = -inf;
                    def.lastInterceptPose = [nan nan nan];
                end

                if all(isfinite(def.lastInterceptPose))
                    interceptMoved = norm(interceptPose(1:2) - def.lastInterceptPose(1:2)) >= replanDist;
                else
                    interceptMoved = true;
                end

                needReplan = (isempty(def.path) || (obj.tick - def.lastPlanTick) >= replanEveryTicks || interceptMoved);

                if needReplan
                    startPose = [def.location def.heading];
                    try
                        pathObj = plan(def.planner, startPose, interceptPose);
                        if isprop(pathObj, "States") && ~isempty(pathObj.States)
                            def.path = pathObj.States;
                            if size(def.path, 1) >= 2
                                def.pathIdx = 2;
                            else
                                def.pathIdx = 1;
                            end
                        else
                            def.path = [];
                            def.pathIdx = 1;
                        end
                    catch
                        def.path = [];
                        def.pathIdx = 1;
                    end

                    def.lastPlanTick = obj.tick;
                    def.lastInterceptPose = interceptPose;
                end

                moved = false;

                if ~isempty(def.path)
                    idx = max(1, min(def.pathIdx, size(def.path, 1)));
                    nextPose = def.path(idx, :);

                    if all(isfinite(nextPose))
                        def.location = nextPose(1:2);
                        def.heading = nextPose(3);

                        def.pathIdx = def.pathIdx + 1;
                        if def.pathIdx > size(def.path, 1)
                            def.path = [];
                            def.pathIdx = 1;
                        end
                        moved = true;
                    else
                        def.path = [];
                        def.pathIdx = 1;
                    end
                end

                if ~moved
                    goal = interceptPose(1:2);
                    v = goal - def.location(1:2);
                    nv = norm(v);

                    if nv > 1e-9
                        dir = v / nv;
                        candDirs = [dir; [-dir(2), dir(1)]; [dir(2), -dir(1)]; -dir];

                        newLoc = def.location(1:2);
                        for c = 1:size(candDirs, 1)
                            trial = def.location(1:2) + step * candDirs(c, :);
                            trial(1) = min(max(trial(1), xMin), xMax);
                            trial(2) = min(max(trial(2), yMin), yMax);

                            ok = true;
                            try
                                occ = getOccupancy(obj.occMap, trial, "world");
                                ok = (occ < 0.5);
                            catch
                            end

                            if ok
                                newLoc = trial;
                                def.heading = atan2(candDirs(c, 2), candDirs(c, 1));
                                break
                            end
                        end

                        def.location = newLoc;
                    end
                end

                obj.defenders(d) = def;
            end
        end

        function effectorsOut = attachEffectorsToDefenders_(~, effectorsIn, defendersIn)
            effectorsOut = effectorsIn;
            if isempty(effectorsIn) || isempty(defendersIn) || ~isstruct(effectorsIn)
                return
            end

            nE = numel(effectorsIn);
            nD = numel(defendersIn);

            for k = 1:nE
                didx = [];
                if isfield(effectorsOut, "defenderIdx") && ~isempty(effectorsOut(k).defenderIdx)
                    didx = effectorsOut(k).defenderIdx;
                elseif isfield(effectorsOut, "defenderIndex") && ~isempty(effectorsOut(k).defenderIndex)
                    didx = effectorsOut(k).defenderIndex;
                elseif isfield(effectorsOut, "defenderID") && ~isempty(effectorsOut(k).defenderID)
                    didx = effectorsOut(k).defenderID;
                elseif nE == nD
                    didx = k;
                end

                if isempty(didx) || didx < 1 || didx > nD
                    continue
                end

                effectorsOut(k).location = defendersIn(didx).location;
            end
        end

        function interceptPose = predictIntercept_(~, ~, uasPosXY, uasObj) 
            lookahead = 2.0;
            vhat = [1 0];

            if isprop(uasObj, "targetUnitVector")
                v = uasObj.targetUnitVector(1:2);
                if norm(v) > 0
                    vhat = v / norm(v);
                end
            end

            spd = 0;
            if isprop(uasObj, "speed")
                spd = uasObj.speed;
            end

            ip = uasPosXY + vhat * spd * lookahead;
            hdg = atan2(vhat(2), vhat(1));
            interceptPose = [ip, hdg];
        end

        function launchKineticFermiShots_(obj, currentTime, uasActive, uasTracked)
            if isempty(obj.effectors) || isempty(obj.UAS)
                return
            end

            for e = 1:numel(obj.effectors)
                if (currentTime - obj.effectorLastFireTime(e)) < obj.kineticRefireTime
                    continue
                end

                effLoc = obj.effectors(e).location(1:2);
                effRange = obj.effectors(e).range;

                if size(obj.effectors3D, 1) >= e && all(isfinite(obj.effectors3D(e, :)))
                    shooterPos3 = obj.effectors3D(e, :);
                else
                    shooterPos3 = obj.getGroundPoint3_(effLoc(1), effLoc(2), simulator.effectorHeight_());
                end

                candidateIdx = [];
                candidateDist = inf;
                candidateTargetPos3 = [];

                for i = 1:numel(obj.UAS)
                    if ~uasActive(i)
                        continue
                    end

                    if nargin >= 4 && ~isempty(uasTracked)
                        if i <= numel(uasTracked) && ~uasTracked(i)
                            continue
                        end
                    end

                    targetObj = obj.UAS(i);
                    targetPos3 = targetObj.position(1:3);
                    d = norm(targetPos3(1:2) - effLoc);

                    if d > effRange
                        continue
                    end

                    if ~obj.hasLineOfSight_(shooterPos3, targetPos3, simulator.losClearance_())
                        continue
                    end

                    if d < candidateDist
                        candidateIdx = i;
                        candidateDist = d;
                        candidateTargetPos3 = targetPos3;
                    end
                end

                if isempty(candidateIdx)
                    continue
                end

                pShot = obj.fermiShotProbability_(candidateDist, effRange);
                pVolley = 1 - (1 - pShot)^obj.kineticShotsPerVolley;
                pVolley = min(1, max(0, pVolley));

                travelTime = norm(candidateTargetPos3 - shooterPos3) / max(obj.kineticProjectileSpeed, eps);
                hitTime = currentTime + travelTime;

                pending = simulator.emptyPendingShot_();
                pending.active = true;
                pending.effectorIdx = e;
                pending.targetIdx = candidateIdx;
                pending.fireTime = currentTime;
                pending.hitTime = hitTime;
                pending.pShot = pShot;
                pending.pVolley = pVolley;
                pending.travelTime = travelTime;
                pending.rangeAtFire = candidateDist;
                pending.willHit = (rand <= pVolley);

                obj.pendingKineticHits(end + 1, 1) = pending; 
                obj.effectorLastFireTime(e) = currentTime;

                fprintf("t=%.2f: Effector %d FIRED kinetic Fermi volley at UAS %d | dist=%.2f m | Pshot=%.3f | Pvolley=%.3f | TOF=%.2f s\n", ...
                    currentTime, e, candidateIdx, candidateDist, pShot, pVolley, travelTime);
            end
        end

        function pShot = fermiShotProbability_(obj, distanceToTarget, effectorRange)
            d50 = max(0.5, obj.kineticFermiD50Scale * effectorRange);
            k = max(0.10, obj.kineticFermiSharpnessScale * effectorRange);
            pShot = obj.kineticHitProbability / (1 + exp((distanceToTarget - d50) / k));
            pShot = min(1, max(0, pShot));
        end

        function [uasActive, killedIdx, killedPos] = stepPendingKineticHits_(obj, currentTime, uasActive)
            killedIdx = zeros(0, 1);
            killedPos = zeros(0, 3);

            if isempty(obj.pendingKineticHits)
                return
            end

            keep = true(numel(obj.pendingKineticHits), 1);

            for sIdx = 1:numel(obj.pendingKineticHits)
                shot = obj.pendingKineticHits(sIdx);
                if ~shot.active
                    keep(sIdx) = false;
                    continue
                end

                if currentTime < shot.hitTime
                    obj.pendingKineticHits(sIdx) = shot;
                    continue
                end

                keep(sIdx) = false;

                tIdx = shot.targetIdx;
                if tIdx < 1 || tIdx > numel(obj.UAS)
                    obj.pendingKineticHits(sIdx) = shot;
                    continue
                end

                if ~uasActive(tIdx)
                    obj.pendingKineticHits(sIdx) = shot;
                    continue
                end

                if shot.willHit
                    uasActive(tIdx) = false;
                    killedIdx(end + 1, 1) = tIdx; 
                    killedPos(end + 1, :) = obj.UAS(tIdx).position; 
                end

                obj.pendingKineticHits(sIdx) = shot;
            end

            obj.pendingKineticHits = obj.pendingKineticHits(keep);
        end

        function launchKineticProjectiles_(obj, currentTime, uasActive, uasTracked) 
            if isempty(obj.effectors) || isempty(obj.UAS)
                return
            end

            for e = 1:numel(obj.effectors)
                if (currentTime - obj.effectorLastFireTime(e)) < obj.kineticRefireTime
                    continue
                end

                effLoc = obj.effectors(e).location(1:2);
                effRange = obj.effectors(e).range;

                if size(obj.effectors3D, 1) >= e && all(isfinite(obj.effectors3D(e, :)))
                    shooterPos3 = obj.effectors3D(e, :);
                else
                    shooterPos3 = obj.getGroundPoint3_(effLoc(1), effLoc(2), simulator.effectorHeight_());
                end

                candidateIdx = [];
                candidateDist = inf;
                candidateAimPoint = [];

                for i = 1:numel(obj.UAS)
                    if ~uasActive(i)
                        continue
                    end

                    uasPos = obj.UAS(i).position(1:2);
                    d = norm(uasPos - effLoc);

                    if d > effRange
                        continue
                    end

                    targetObj = obj.UAS(i);
                    targetPos3 = targetObj.position(1:3);

                    if ~obj.hasLineOfSight_(shooterPos3, targetPos3, simulator.losClearance_())
                        continue
                    end

                    [aimPoint3, canSolve] = obj.solveLinearIntercept_(shooterPos3, targetPos3, targetObj);
                    if ~canSolve
                        aimPoint3 = targetPos3;
                    end

                    if d < candidateDist
                        candidateIdx = i;
                        candidateDist = d;
                        candidateAimPoint = aimPoint3;
                    end
                end

                if isempty(candidateIdx)
                    continue
                end

                dir3 = candidateAimPoint - shooterPos3;
                nDir = norm(dir3);
                if nDir < eps
                    continue
                end
                vel3 = obj.kineticProjectileSpeed * (dir3 / nDir);

                fprintf("t=%.2f: Effector %d FIRED kinetic volley at UAS %d | dist=%.2f m\n", ...
                    currentTime, e, candidateIdx, candidateDist);

                for s = 1:obj.kineticShotsPerVolley
                    p = simulator.emptyProjectile_();
                    p.active = true;
                    p.pos = shooterPos3;
                    p.prevPos = shooterPos3;
                    p.vel = vel3;
                    p.targetIdx = candidateIdx;
                    p.hitEligible = (rand <= obj.kineticHitProbability);
                    p.missOffset = [0 0 0];
                    p.life = 0;
                    p.maxLife = max(1.0, 2.0 * max(norm(candidateAimPoint - shooterPos3), eps) / obj.kineticProjectileSpeed);
                    p.trail = shooterPos3;

                    if ~p.hitEligible
                        missDir = randn(1, 3);
                        if norm(missDir) > 0
                            missDir = missDir / norm(missDir);
                        else
                            missDir = [1 0 0];
                        end
                        p.missOffset = missDir * max(1.5, obj.projectileHitTolerance * 2);
                    end

                    obj.projectiles(end + 1, 1) = p; 
                end

                obj.effectorLastFireTime(e) = currentTime;
            end
        end

        function [uasActive, killedIdx, killedPos] = stepProjectiles_(obj, dtLocal, uasActive)
            killedIdx = zeros(0, 1);
            killedPos = zeros(0, 3);

            if isempty(obj.projectiles)
                return
            end

            keep = true(numel(obj.projectiles), 1);

            for pIdx = 1:numel(obj.projectiles)
                p = obj.projectiles(pIdx);
                if ~p.active
                    keep(pIdx) = false;
                    continue
                end

                oldPos = p.pos;
                p.prevPos = oldPos;
                p.pos = p.pos + p.vel * dtLocal;
                p.life = p.life + dtLocal;
                p.trail = [p.trail; p.pos]; 

                x = p.pos(1);
                y = p.pos(2);

                xMin = obj.occMap.XWorldLimits(1);
                xMax = obj.occMap.XWorldLimits(2);
                yMin = obj.occMap.YWorldLimits(1);
                yMax = obj.occMap.YWorldLimits(2);

                if x < xMin || x > xMax || y < yMin || y > yMax
                    keep(pIdx) = false;
                    obj.projectiles(pIdx) = p;
                    continue
                end

                if obj.segmentHitsTerrain_(oldPos, p.pos, 0.02)
                    keep(pIdx) = false;
                    obj.projectiles(pIdx) = p;
                    continue
                end

                if p.life > p.maxLife
                    keep(pIdx) = false;
                    obj.projectiles(pIdx) = p;
                    continue
                end

                tIdx = p.targetIdx;
                if tIdx < 1 || tIdx > numel(obj.UAS) || ~uasActive(tIdx)
                    keep(pIdx) = false;
                    obj.projectiles(pIdx) = p;
                    continue
                end

                targetPos = obj.UAS(tIdx).position(1:3);
                if ~p.hitEligible
                    targetPos = targetPos + p.missOffset;
                end

                dSeg = simulator.pointToSegmentDistance3D_(targetPos, oldPos, p.pos);
                if dSeg <= obj.projectileHitTolerance
                    keep(pIdx) = false;
                    if p.hitEligible && uasActive(tIdx)
                        uasActive(tIdx) = false;
                        killedIdx(end + 1, 1) = tIdx; 
                        killedPos(end + 1, :) = obj.UAS(tIdx).position; 
                    end
                end

                obj.projectiles(pIdx) = p;
            end

            obj.projectiles = obj.projectiles(keep);
        end

        function updateProjectileGraphics_(obj)
            obj.clearProjectileGraphics_();

            if isempty(obj.projectiles)
                return
            end

            ax = [];
            try
                ax = obj.map.axSim;
            catch
                ax = [];
            end

            if isempty(ax) || ~isgraphics(ax, 'axes')
                ax = gca;
            end

            holdState = ishold(ax);
            hold(ax, 'on');

            nProj = numel(obj.projectiles);
            obj.projectileHandles = gobjects(nProj, 1);
            obj.projectileTrailHandles = gobjects(nProj, 1);

            for k = 1:nProj
                pos = obj.projectiles(k).pos;
                trail = obj.projectiles(k).trail;

                if ~obj.grid2D
                    obj.projectileTrailHandles(k) = plot3(ax, trail(:, 1), trail(:, 2), trail(:, 3), ...
                        'r-', 'LineWidth', 2);

                    obj.projectileHandles(k) = plot3(ax, pos(1), pos(2), pos(3), ...
                        'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                else
                    obj.projectileTrailHandles(k) = plot(ax, trail(:, 1), trail(:, 2), ...
                        'r-', 'LineWidth', 2);

                    obj.projectileHandles(k) = plot(ax, pos(1), pos(2), ...
                        'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                end
            end

            if ~holdState
                hold(ax, 'off');
            end
        end

        function clearProjectileGraphics_(obj)
            if ~isempty(obj.projectileHandles)
                for k = 1:numel(obj.projectileHandles)
                    if isgraphics(obj.projectileHandles(k))
                        delete(obj.projectileHandles(k));
                    end
                end
            end

            if ~isempty(obj.projectileTrailHandles)
                for k = 1:numel(obj.projectileTrailHandles)
                    if isgraphics(obj.projectileTrailHandles(k))
                        delete(obj.projectileTrailHandles(k));
                    end
                end
            end

            obj.projectileHandles = gobjects(0);
            obj.projectileTrailHandles = gobjects(0);
        end

        function renderTerrain3D_(obj)
            if obj.grid2D
                return
            end

            ax = [];
            try
                ax = obj.map.axSim;
            catch
                ax = [];
            end

            if isempty(ax) || ~isgraphics(ax, 'axes')
                return
            end

            if isgraphics(obj.terrainHandle)
                xlabel(ax, 'X (m)');
                ylabel(ax, 'Y (m)');
                zlabel(ax, 'Elevation (m)');
                return
            end

            xMin = obj.occMap.XWorldLimits(1);
            xMax = obj.occMap.XWorldLimits(2);
            yMin = obj.occMap.YWorldLimits(1);
            yMax = obj.occMap.YWorldLimits(2);

            nGrid = 45;
            x = linspace(xMin, xMax, nGrid);
            y = linspace(yMin, yMax, nGrid);
            [X, Y] = meshgrid(x, y);
            Z = zeros(size(X));

            for r = 1:size(X, 1)
                for c = 1:size(X, 2)
                    Z(r, c) = obj.map.getElevation(X(r, c), Y(r, c));
                end
            end

            holdState = ishold(ax);
            hold(ax, 'on');

            obj.terrainHandle = surf(ax, X, Y, Z, ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.65);

            view(ax, 3);
            xlabel(ax, 'X (m)');
            ylabel(ax, 'Y (m)');
            zlabel(ax, 'Elevation (m)');
            grid(ax, 'on');

            if ~holdState
                hold(ax, 'off');
            end
        end

        function update3DTitle_(obj, currentTime, nIntercept, nActive, nAssetHit, nCrash, nNFZ)
            ax = [];
            try
                ax = obj.map.axSim;
            catch
                ax = [];
            end

            if isempty(ax) || ~isgraphics(ax, 'axes')
                return
            end

            title(ax, sprintf(['UAS Simulation (3D Terrain & Coverage)\n' ...
                't = %.2fs | Intercepted=%d | Active=%d | AssetHit=%d | Crash=%d | NFZ=%d'], ...
                currentTime, nIntercept, nActive, nAssetHit, nCrash, nNFZ), ...
                'FontWeight', 'bold');
            xlabel(ax, 'X (m)');
            ylabel(ax, 'Y (m)');
            zlabel(ax, 'Elevation (m)');
        end

        function [aimPoint, canSolve] = solveLinearIntercept_(obj, shooterPos, targetPos, uasObj)
            canSolve = false;
            aimPoint = targetPos;

            if isprop(uasObj, "targetUnitVector")
                vhat = uasObj.targetUnitVector(1:2);
                nv = norm(vhat);
                if nv > 0
                    vhat = vhat / nv;
                else
                    vhat = [1 0];
                end
            else
                vhat = [1 0];
            end

            vTarget = [0 0 0];
            if isprop(uasObj, "speed")
                vTarget = [uasObj.speed * vhat, 0];
            end

            r = targetPos - shooterPos;
            s = obj.kineticProjectileSpeed;

            a = dot(vTarget, vTarget) - s^2;
            b = 2 * dot(r, vTarget);
            c = dot(r, r);

            if abs(a) < 1e-12
                if abs(b) < 1e-12
                    t = 0;
                else
                    t = -c / b;
                end
                if t > 0
                    aimPoint = targetPos + vTarget * t;
                    canSolve = true;
                end
                return
            end

            disc = b^2 - 4 * a * c;
            if disc < 0
                return
            end

            t1 = (-b + sqrt(disc)) / (2 * a);
            t2 = (-b - sqrt(disc)) / (2 * a);
            tCandidates = [t1 t2];
            tCandidates = tCandidates(tCandidates > 0);

            if isempty(tCandidates)
                return
            end

            t = min(tCandidates);
            aimPoint = targetPos + vTarget * t;
            canSolve = true;
        end

        function p3 = getGroundPoint3_(obj, x, y, zOffset)
            if nargin < 5
                zOffset = 0;
            end
            z = obj.map.getElevation(x, y);
            p3 = [x, y, z + zOffset];
        end

        function tf = hasLineOfSight_(obj, p0, p1, clearance)
            if nargin < 4 || isempty(clearance)
                clearance = 0;
            end

            n = max(25, ceil(norm(p1(1:2) - p0(1:2)) / 0.75));
            s = linspace(0, 1, n);

            x = p0(1) + s * (p1(1) - p0(1));
            y = p0(2) + s * (p1(2) - p0(2));
            z = p0(3) + s * (p1(3) - p0(3));

            zTerr = obj.map.getElevation(x, y);
            tf = all(z >= (zTerr + clearance));
        end

        function tf = segmentHitsTerrain_(obj, p0, p1, clearance)
            if nargin < 4 || isempty(clearance)
                clearance = 0;
            end

            n = max(10, ceil(norm(p1(1:2) - p0(1:2)) / 0.5));
            s = linspace(0, 1, n);

            x = p0(1) + s * (p1(1) - p0(1));
            y = p0(2) + s * (p1(2) - p0(2));
            z = p0(3) + s * (p1(3) - p0(3));

            zTerr = obj.map.getElevation(x, y);
            tf = any(z <= (zTerr + clearance));
        end
    end

    methods (Static, Access = private)
        function out = normalizeDefenderStructArray_(in)
            defaults = struct('location', [0 0], 'speed', 10, 'mode', "HYBRIDA*", 'heading', 0, 'planner', [], 'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);

            n = numel(in);
            out = repmat(defaults, n, 1);

            for i = 1:n
                di = in(i);
                fni = fieldnames(di);
                for k = 1:numel(fni)
                    f = fni{k};
                    if isfield(out(i), f)
                        out(i).(f) = di.(f);
                    end
                end

                try
                    out(i).location = out(i).location(:).';
                    out(i).location = out(i).location(1:2);
                catch
                    out(i).location = defaults.location;
                end

                if isempty(out(i).speed) || out(i).speed <= 0
                    out(i).speed = defaults.speed;
                end
                if isempty(out(i).mode)
                    out(i).mode = defaults.mode;
                end
            end
        end

        function NFZplot = unionPolys(polys)
            NFZplot = polyshape.empty;
            if isempty(polys)
                return
            end

            NFZplot = polys(1);
            for k = 2:numel(polys)
                NFZplot = union(NFZplot, polys(k));
            end
        end

        function eff3D = buildEffectors3D_(mapObj, effectors, doUpdate)
            if nargin < 3
                doUpdate = false;
            end

            if isempty(effectors)
                eff3D = [];
                return
            end

            numEff = numel(effectors);
            eff3D = zeros(numEff, 3);

            for k = 1:numEff
                if doUpdate && ~isstruct(effectors) && ismethod(effectors(k), "update")
                    effectors(k).update();
                end

                if ~isfield(effectors(k), "location") || isempty(effectors(k).location)
                    loc = [nan nan];
                else
                    loc = effectors(k).location;
                end

                if any(~isfinite(loc(1:2)))
                    eff3D(k, :) = [nan nan nan];
                    continue
                end

                z = mapObj.getElevation(loc(1), loc(2)) + simulator.effectorHeight_();
                eff3D(k, :) = [loc(1), loc(2), z];
            end
        end

        function p = emptyProjectile_()
            p = struct( ...
                'active', false, ...
                'pos', [0 0 0], ...
                'prevPos', [0 0 0], ...
                'vel', [0 0 0], ...
                'targetIdx', 0, ...
                'hitEligible', false, ...
                'missOffset', [0 0 0], ...
                'life', 0, ...
                'maxLife', 0, ...
                'trail', zeros(0, 3));
        end

        function s = emptyPendingShot_()
            s = struct( ...
                'active', false, ...
                'effectorIdx', 0, ...
                'targetIdx', 0, ...
                'fireTime', 0, ...
                'hitTime', 0, ...
                'pShot', 0, ...
                'pVolley', 0, ...
                'travelTime', 0, ...
                'rangeAtFire', 0, ...
                'willHit', false);
        end

        function d = pointToSegmentDistance3D_(pt, a, b)
            ab = b - a;
            denom = dot(ab, ab);

            if denom <= eps
                d = norm(pt - a);
                return
            end

            t = dot(pt - a, ab) / denom;
            t = max(0, min(1, t));
            proj = a + t * ab;
            d = norm(pt - proj);
        end

        function h = effectorHeight_()
            h = 1.2;
        end

        function h = defenderHeight_()
            h = 0.8;
        end

        function c = losClearance_()
            c = 0.10;
        end
    end
end
