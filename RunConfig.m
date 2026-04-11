function [Result] = RunConfig(scenario, mapObj, asset, sensors, currentSeeds, effector_posns_bank, adversary_paths_bank, mcSettings, costConfig, simConfig, weaponConfig, advConfig)
    advConfig.count = scenario.advCount;
    totalEff = scenario.effCount;
    
    if strcmpi(scenario.weaponType, "Kinetic")
        weaponTypes = repmat("KINETIC", 1, totalEff);
    elseif strcmpi(scenario.weaponType, "DirectEnergy")
        weaponTypes = repmat("DIRECT_ENERGY", 1, totalEff);
    elseif strcmpi(scenario.weaponType, "Mixed")
        numKin = floor(totalEff / 2);
        numDE = totalEff - numKin; 
        weaponTypes = [repmat("KINETIC", 1, numKin), repmat("DIRECT_ENERGY", 1, numDE)];
    else
        weaponTypes = repmat("LEGACY", 1, totalEff); 
    end

    if strcmpi(scenario.mobilityType, "Static")
        numStatic = totalEff; numMobile = 0;
    elseif strcmpi(scenario.mobilityType, "Mobile")
        numStatic = 0; numMobile = totalEff;
    elseif strcmpi(scenario.mobilityType, "Mixed")
        numStatic = floor(totalEff / 2); numMobile = ceil(totalEff / 2);
    end

    effectorStructTemplate = struct('location', [0,0], 'range', weaponConfig.effectorRange,...
        'mode', "STATIC", 'weaponMode', "LEGACY", 'speed', 0, 'heading', 0, 'planner', [], ...
        'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);
        
    randIndices = randperm(size(effector_posns_bank, 1), totalEff);
    effPos = effector_posns_bank(randIndices, :);
    
    weaponTypes = weaponTypes(randperm(length(weaponTypes)));
    
    mobilityTypes = [repmat("STATIC", 1, numStatic), repmat("MOBILE", 1, numMobile)];
    mobilityTypes = mobilityTypes(randperm(length(mobilityTypes)));

    currentEffectors = repmat(effectorStructTemplate, totalEff, 1);
    for e = 1:totalEff
        currentEffectors(e).location = effPos(e, :);
        currentEffectors(e).weaponMode = weaponTypes(e); 
        currentEffectors(e).mode = mobilityTypes(e); 
        
        if currentEffectors(e).mode == "STATIC"
            currentEffectors(e).speed = 0;
        else
            currentEffectors(e).speed = weaponConfig.mobileSpeed; 
        end
    end

    mapL = mapObj.size.vert;
    mapW = mapObj.size.horiz;
    flightAlt = advConfig.altitude;
    [X, Y] = meshgrid(0:mapW, 0:mapL);
    Z_raw = mapObj.terrainProxy(X', Y'); 
    Z = Z_raw';
    costmap = Z >= flightAlt;
    costmapMatrix = flipud(costmap); % Store as raw logical array

    runCosts = zeros(mcSettings.maxTests, 1);
    runFailures = zeros(mcSettings.maxTests, 1);
    runKills = cell(mcSettings.maxTests, 1);
    
    testsCompleted = 0;
    ciConverged = false;
    
    while testsCompleted < mcSettings.maxTests && ~ciConverged
        if testsCompleted == 0
            testsThisRound = mcSettings.minTests;
        else
            testsThisRound = min(mcSettings.testChunkSize, mcSettings.maxTests - testsCompleted);
        end
        
        chunkCosts = zeros(testsThisRound, 1);
        chunkFailures = zeros(testsThisRound, 1);
        chunkKills = cell(testsThisRound, 1);
        
        if simConfig.parallel
            parfor j = 1:testsThisRound
                trialIdx = testsCompleted + j;
                localStream = RandStream('mt19937ar', 'Seed', currentSeeds(trialIdx));                

                uasArray = UAS.empty(0, advConfig.count); 
                for k = 1:advConfig.count
                    pIdx = randi(localStream, length(adversary_paths_bank)); 
                    path = adversary_paths_bank{1,1,pIdx};
                    startX = path(1, 1); startY = path(1, 2); startZ = advConfig.altitude;
                    groundElevation = mapObj.getElevation(startX, startY);
                    while groundElevation >= startZ; startZ = startZ + 1; end
                    startPos = [startX, startY, startZ];
                    uasArray(k) = UAS(advConfig.speed, startPos, asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path, "searchRange", advConfig.searchRange);
                end
                
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'resetGraphics', false, 'costConfig', costConfig, ...
                    'randomStream', localStream, 'costmapMatrix', costmapMatrix, ... 
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                
                chunkCosts(j) = runResults.cost; 
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); chunkKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; chunkFailures(j) = 1; end

                delete(sim);
                delete(uasArray);
            end
        else
            for j = 1:testsThisRound
                trialIdx = testsCompleted + j;
                localStream = RandStream('mt19937ar', 'Seed', currentSeeds(trialIdx)); 

                uasArray = UAS.empty(0, advConfig.count); 
                for k = 1:advConfig.count
                    pIdx = randi(localStream, length(adversary_paths_bank)); 
                    path = adversary_paths_bank{1,1,pIdx};
                    startX = path(1, 1); startY = path(1, 2); startZ = advConfig.altitude;
                    groundElevation = mapObj.getElevation(startX, startY);
                    while groundElevation >= startZ; startZ = startZ + 1; end
                    startPos = [startX, startY, startZ];
                    uasArray(k) = UAS(advConfig.speed, startPos, asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path, "searchRange", advConfig.searchRange);
                end
                
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'resetGraphics', false, 'costConfig', costConfig, ...
                    'randomStream', localStream, 'costmapMatrix', costmapMatrix, ... 
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                
                chunkCosts(j) = runResults.cost; 
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); chunkKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; chunkFailures(j) = 1; end

                delete(sim);
                delete(uasArray);
            end
        end
        
        idxRange = (testsCompleted + 1) : (testsCompleted + testsThisRound);
        runCosts(idxRange) = chunkCosts;
        runFailures(idxRange) = chunkFailures;
        runKills(idxRange) = chunkKills;
        
        testsCompleted = testsCompleted + testsThisRound;
        
        if testsCompleted > 1
            currSE = std(runCosts(1:testsCompleted)) / sqrt(testsCompleted);
            tcrit = tinv(1 - mcSettings.confidenceAlpha/2, testsCompleted - 1);
            ciWidth = 2 * tcrit * currSE;
            
            if ciWidth <= mcSettings.targetCIWidth
                ciConverged = true;
            end
        end
    end
    
    Result.runCosts = runCosts(1:testsCompleted);
    Result.runFailures = sum(runFailures(1:testsCompleted));
    Result.runKills = vertcat(runKills{1:testsCompleted}); 
    Result.testsCompleted = testsCompleted;
    Result.Effectors = currentEffectors;
end