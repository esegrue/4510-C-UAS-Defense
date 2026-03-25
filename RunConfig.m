function [Result] = RunConfig(scenario, mapObj, asset, sensors, currentSeeds, effector_posns_bank, adversary_paths_bank, mcSettings, costConfig, simConfig, weaponConfig, advConfig)
% Evaluates a SINGLE configuration for a specific scenario using Adaptive Sampling

    % 1. OVERRIDE CONFIGS BASED ON SCENARIO
    advConfig.count = scenario.advCount;
    totalEff = scenario.effCount;
    
    % Mobility Logic
    if strcmpi(scenario.mobilityType, "Static")
        numStatic = totalEff; numMobile = 0;
    elseif strcmpi(scenario.mobilityType, "Mobile")
        numStatic = 0; numMobile = totalEff;
    elseif strcmpi(scenario.mobilityType, "Mixed")
        numStatic = floor(totalEff / 2); numMobile = ceil(totalEff / 2);
    end

    % Weapon Logic
    if strcmpi(scenario.weaponType, "Kinetic")
        numKinetic = totalEff; numDE = 0;
    elseif strcmpi(scenario.weaponType, "DirectEnergy")
        numKinetic = 0; numDE = totalEff;
    elseif strcmpi(scenario.weaponType, "Mixed")
        numKinetic = floor(totalEff / 2); numDE = ceil(totalEff / 2);
    end
    
    weaponTypes = [repmat("DIRECT_ENERGY", 1, numDE), repmat("KINETIC", 1, numKinetic)];

    % 2. GENERATE EFFECTORS
    effectorStructTemplate = struct('location', [0,0], 'range', 20, ...
        'mode', "STATIC", 'weaponMode', "LEGACY", 'speed', 0, 'heading', 0, 'planner', [], ...
        'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);
        
    randIndices = randperm(size(effector_posns_bank, 1), totalEff);
    effPos = effector_posns_bank(randIndices, :);
    
    currentEffectors = repmat(effectorStructTemplate, totalEff, 1);
    for e = 1:totalEff
        currentEffectors(e).location = effPos(e, :);
        currentEffectors(e).weaponMode = weaponTypes(e); 
        if e <= numStatic
            currentEffectors(e).mode = "STATIC"; currentEffectors(e).speed = 0;
        else
            currentEffectors(e).mode = "MOBILE"; currentEffectors(e).speed = 12; % mobile speed
        end
    end

    % 3. INITIALIZE ADAPTIVE STORAGE
    runCosts = zeros(mcSettings.maxTests, 1); 
    runStarts = cell(mcSettings.maxTests, 1);
    runPaths = cell(mcSettings.maxTests, 1);
    runFailures = zeros(mcSettings.maxTests, 1);
    runKills = cell(mcSettings.maxTests, 1);
    
    testsCompleted = 0;
    ciConverged = false;
    
    % 4. RUN ADAPTIVE CHUNKING
    while testsCompleted < mcSettings.maxTests && ~ciConverged
        if testsCompleted == 0
            testsThisRound = mcSettings.minTests;
        else
            testsThisRound = min(mcSettings.testChunkSize, mcSettings.maxTests - testsCompleted);
        end
        
        chunkCosts = zeros(testsThisRound, 1);
        chunkStarts = cell(testsThisRound, 1);
        chunkPaths = cell(testsThisRound, 1);
        chunkFailures = zeros(testsThisRound, 1);
        chunkKills = cell(testsThisRound, 1);
        
        if simConfig.parallel
            parfor j = 1:testsThisRound
                trialIdx = testsCompleted + j;
                rng(currentSeeds(trialIdx), 'twister');
                
                uasArray = UAS.empty(0, advConfig.count); pathIdxs = zeros(advConfig.count, 1); starts = zeros(advConfig.count, 3);
                for k = 1:advConfig.count
                    pIdx = randi(length(adversary_paths_bank)); pathIdxs(k) = pIdx; path = adversary_paths_bank{1,1,pIdx};
                    startX = path(1, 1); startY = path(1, 2); startZ = advConfig.altitude;
                    groundElevation = mapObj.getElevation(startX, startY);
                    while groundElevation >= startZ; startZ = startZ + 1; end
                    starts(k, :) = [startX, startY, startZ];
                    uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path);
                end
                
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'resetGraphics', false, 'costConfig', costConfig, ...
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                
                chunkCosts(j) = runResults.cost; 
                chunkStarts{j} = starts; 
                chunkPaths{j} = pathIdxs;
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); chunkKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; chunkFailures(j) = 1; end
            end
        end
        
        idxRange = (testsCompleted + 1) : (testsCompleted + testsThisRound);
        runCosts(idxRange) = chunkCosts;
        runStarts(idxRange) = chunkStarts;
        runPaths(idxRange) = chunkPaths;
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
    
    % 5. PACKAGE RESULTS
    Result.runCosts = runCosts(1:testsCompleted);
    Result.runStarts = runStarts(1:testsCompleted);
    Result.runPaths = runPaths(1:testsCompleted);
    Result.runFailures = sum(runFailures(1:testsCompleted));
    Result.runKills = vertcat(chunkKills{:}); 
    Result.testsCompleted = testsCompleted;
    Result.Effectors = currentEffectors;
end