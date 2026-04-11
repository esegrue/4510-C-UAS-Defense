clear; clc; close all;

%%  USER CONFIGURATION
%  -------------------
% OPTIMIZATION SETTINGS
mcSettings.maxConfigs = 5; 
mcSettings.batchSize = 2; 
mcSettings.searchPatience = 2; % number of iterations without beating rival
mcSettings.minSeparation = 50; % minimum separation
mcSettings.confidenceAlpha = 0.1; % confidence in average cost margin (1-alpha)
mcSettings.reliabilityThresh = 90; % percent of successful tests

% ADAPTIVE SAMPLING SETTINGS
mcSettings.minTests = 5;         
mcSettings.maxTests = 50;         
mcSettings.testChunkSize = 5; 
mcSettings.targetCIWidth = 100; 

% MAP SETTINGS
mapConfig.L = 100; 
mapConfig.W = 100; 
mapConfig.pathBankFile = "adversary_paths_bank.mat";

% ASSET LOCATION
assetConfig.location = [30, 60]; 

% EFFECTOR SETTINGS
effConfig.numStatic = 3; 
effConfig.numMobile = 0; 
effConfig.mobileSpeed = 12; 
effConfig.range = 20; 
effConfig.posBankFile = "effector_posn_bank.mat";

% WEAPON SETTINGS (Heterogeneous Loadouts)
weaponConfig.numLegacy = 0;       
weaponConfig.numDirectEnergy = 1; 
weaponConfig.numKinetic = 2;      
weaponConfig.directEnergyDwellTime = 0.75;
weaponConfig.kineticProjectileSpeed = 30;
weaponConfig.kineticShotsPerVolley = 3;
weaponConfig.kineticHitProbability = 1.0;
weaponConfig.kineticRefireTime = 0.50;
weaponConfig.projectileHitTolerance = 1.0;
weaponConfig.kineticUseFermiModel = true; 

% ADVERSARY SETTINGS
advConfig.count = 3; 
advConfig.speed = 15; 
advConfig.turnRadius = 2; 
advConfig.altitude = 15; 
advConfig.planner = 'HybridAStar'; 
advConfig.searchRange = 20;

% SENSOR SETTINGS
sensConfig.count = 3; 
sensConfig.locations = [30,85; 8.35,47.5; 51.65,47.5]; 
sensConfig.range = 20; 
sensConfig.params = struct('d50', sensConfig.range, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

% COST FUNCTION SETTINGS
costConfig.effector = 100; 
costConfig.asset = 2000; 
costConfig.leak = 250; 

% SIMULATION ENGINE SETTINGS
simConfig.parallel = false; 
simConfig.numCores = feature("numcores")/2;
simConfig.tps = 20; 
simConfig.animateLive = false; 

%% SIMULATION SETUP
% -----------------
rng('shuffle')
load big_island_map.mat
mapObj = elevationMap;
mapBounds = [0 mapConfig.L 0 mapConfig.W];
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.maxTests);
configStore = cell(mcSettings.maxConfigs, 1);      
costDetailsStore = cell(mcSettings.maxConfigs, 1); 
killStore = cell(mcSettings.maxConfigs, 1);

CostperCombo = nan(mcSettings.maxConfigs, 1);
ReliabilityScore = nan(mcSettings.maxConfigs, 1); 
LCB = nan(mcSettings.maxConfigs, 1); 
UCB = nan(mcSettings.maxConfigs, 1); 
separation = nan(mcSettings.maxConfigs, 1);

if isfile(effConfig.posBankFile)
    load(effConfig.posBankFile, 'effector_posns_bank');
else
    error('Effector position bank file not found: %s', effConfig.posBankFile);
end
if isfile(mapConfig.pathBankFile)
    load(mapConfig.pathBankFile, 'adversary_paths_bank');
else
    error('Adversary path bank file not found: %s', mapConfig.pathBankFile);
end

asset = struct('location', assetConfig.location);

sensorStructTemplate = struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params); 
sensors = repmat(sensorStructTemplate, sensConfig.count, 1);
for i = 1:sensConfig.count
    if i <= size(sensConfig.locations, 1)
        sensors(i).location = sensConfig.locations(i,:);
    else
        warning('More sensors requested than locations provided. Sensor %d at [0,0].', i);
    end
end

effConfig.totalEffectors = effConfig.numStatic + effConfig.numMobile;
weaponTypes = [repmat("LEGACY", 1, weaponConfig.numLegacy), ...
               repmat("DIRECT_ENERGY", 1, weaponConfig.numDirectEnergy), ...
               repmat("KINETIC", 1, weaponConfig.numKinetic)];
               
if length(weaponTypes) ~= effConfig.totalEffectors
    error('The sum of weapon types must exactly equal effConfig.totalEffectors.');
end
effectorStructTemplate = struct('location', [0,0], 'range', effConfig.range, ...
    'mode', "STATIC", 'weaponMode', "LEGACY", 'speed', 0, 'heading', 0, 'planner', [], ...
    'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);

%% MONTE CARLO SIMULATION
% -----------------------
numConfigs = 0;
batchStartIdx = 1; 
configsSinceNewBest = 0;
bestConfigTracker = 0;
fprintf('Starting Adaptive Monte Carlo Simulation...\n');

while numConfigs < mcSettings.maxConfigs
    numConfigs = numConfigs + 1;
    
    randIndices = randperm(size(effector_posns_bank, 1), effConfig.totalEffectors);
    effPos = effector_posns_bank(randIndices, :);
    
    currentEffectors = repmat(effectorStructTemplate, effConfig.totalEffectors, 1);
    for e = 1:effConfig.totalEffectors
        currentEffectors(e).location = effPos(e, :);
        currentEffectors(e).weaponMode = weaponTypes(e); 
        if e <= effConfig.numStatic
            currentEffectors(e).mode = "STATIC";
            currentEffectors(e).speed = 0;
        else
            currentEffectors(e).mode = "MOBILE";
            currentEffectors(e).speed = effConfig.mobileSpeed;
        end
    end
    configStore{numConfigs} = currentEffectors;
    
    % 2. INITIALIZE STORAGE
    runCosts = zeros(mcSettings.maxTests, 1); 
    runFailures = zeros(mcSettings.maxTests, 1);
    runKills = cell(mcSettings.maxTests, 1);
    
    % 3. RUN ADAPTIVE TESTS (CHUNKING)
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
            if isempty(gcp('nocreate')); parpool(simConfig.numCores); end
            parfor j = 1:testsThisRound
                trialIdx = testsCompleted + j;
                localStream = RandStream('mt19937ar', 'Seed', trialSeeds(numConfigs, trialIdx));
                
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
                    'randomStream', localStream, ...
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                
                chunkCosts(j) = runResults.cost; 
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); chunkKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; chunkFailures(j) = 1; end
            end
        else
            for j = 1:testsThisRound
                trialIdx = testsCompleted + j;
                localStream = RandStream('mt19937ar', 'Seed', trialSeeds(numConfigs, trialIdx));
        
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
                
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', simConfig.animateLive, 'resetGraphics', true, 'costConfig', costConfig, ...
                    'randomStream', localStream, ...
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                
                chunkCosts(j) = runResults.cost; 
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); chunkKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; chunkFailures(j) = 1; end
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
    
    runCosts = runCosts(1:testsCompleted);
    runFailures = runFailures(1:testsCompleted);
    runKills = runKills(1:testsCompleted);
    
    currentConfigKills = vertcat(runKills{:}); runFailures = sum(runFailures);
    
    % 4. STORE CONFIGURATION RESULTS
    costDetailsStore{numConfigs} = runCosts;
    CostperCombo(numConfigs) = mean(runCosts);
    killStore{numConfigs} = currentConfigKills;

    % 5. CONVERGENCE
    currentReliability = 100 * (1 - (runFailures / testsCompleted));
    ReliabilityScore(numConfigs) = currentReliability;
    
    if testsCompleted > 1
        SD = std(runCosts, 0); 
        SE = SD/sqrt(testsCompleted); 
        tcrit = tinv(1 - mcSettings.confidenceAlpha/2, testsCompleted - 1);
        LCB(numConfigs) = CostperCombo(numConfigs) - tcrit*SE;
        UCB(numConfigs) = CostperCombo(numConfigs) + tcrit*SE;
    else
        LCB(numConfigs) = CostperCombo(numConfigs);
        UCB(numConfigs) = CostperCombo(numConfigs);
    end
    
    validMask = find(ReliabilityScore(1:numConfigs) >= mcSettings.reliabilityThresh);
    
    if isempty(validMask) || length(validMask) == 1
        separation(numConfigs) = -inf;
        fprintf('Iter %d: %d tests | No comparison (Rel: %.1f%%)\n', numConfigs, testsCompleted, currentReliability);
    else
        validCosts = CostperCombo(validMask);
        [~, bestIdxInValid] = min(validCosts);
        bestGlobalID = validMask(bestIdxInValid);
        
        rivalsMask = validMask(validMask ~= bestGlobalID);
        separation(numConfigs) = min(LCB(rivalsMask)) - UCB(bestGlobalID);
        
        if bestGlobalID ~= bestConfigTracker
            bestConfigTracker = bestGlobalID;
            configsSinceNewBest = 0;
        else
            configsSinceNewBest = configsSinceNewBest + 1;
        end
        
        fprintf('Iter %d: %d tests | Valid Best #%d ($%.0f) vs Rival ($%.0f) | Sep: %.2f\n', ...
            numConfigs, testsCompleted, bestGlobalID, CostperCombo(bestGlobalID), ...
            min(CostperCombo(rivalsMask)), separation(numConfigs));
    end

    % 6. INCREMENTAL BATCH SAVING, ROLLING SUMMARY, & RAM FLUSH
    if mod(numConfigs, mcSettings.batchSize) == 0 || numConfigs == mcSettings.maxConfigs || (configsSinceNewBest >= mcSettings.searchPatience && separation(numConfigs) > mcSettings.minSeparation)
        fprintf('Saving Batch (Configs %d to %d) to disk...\n', batchStartIdx, numConfigs);
        
        SimResults = struct();
        SimResults.Metadata = struct(...
            'Timestamp', datestr(now), 'MapBounds', mapBounds, ...
            'NumAdversaries', advConfig.count, 'NumSensors', sensConfig.count, ...
            'NumEffectors', effConfig.totalEffectors, 'CostConfig', costConfig, ...
            'ReliabilityThreshold', mcSettings.reliabilityThresh, ...
            'SensorParams', sensConfig.params, 'EffectorRange', effConfig.range, ...
            'MCSettings', mcSettings, 'advConfig', advConfig, 'weaponConfig', weaponConfig);

        SimResults.MapData = mapObj; 
        SimResults.Asset = asset; 
        SimResults.Sensors = sensors;
        SimResults.Configs = struct();
        
        idx = 1;
        for i = batchStartIdx:numConfigs
            SimResults.Configs(idx).ID = i;
            SimResults.Configs(idx).Effectors = configStore{i};
            SimResults.Configs(idx).CostMean = CostperCombo(i);
            SimResults.Configs(idx).Reliability = ReliabilityScore(i);
            SimResults.Configs(idx).LCB = LCB(i);
            SimResults.Configs(idx).UCB = UCB(i);
            SimResults.Configs(idx).KillLocations = killStore{i};
            SimResults.Configs(idx).Separation = separation(i); 
            
            SimResults.Configs(idx).Trials = struct();
            trialsRun = length(costDetailsStore{i});
            for t = 1:trialsRun
                SimResults.Configs(idx).Trials(t).Seed = trialSeeds(i, t); 
                SimResults.Configs(idx).Trials(t).Cost = costDetailsStore{i}(t);
            end
            idx = idx + 1;
        end
        
        fileName = sprintf('SimData_Batch_%dto%d_%s.mat', batchStartIdx, numConfigs, datestr(now, 'yyyymmdd_HHMMSS'));
        save(fileName, 'SimResults', '-v7.3');
        
        validCandidates = find(ReliabilityScore(1:numConfigs) >= mcSettings.reliabilityThresh);
        if isempty(validCandidates)
            [~, bestID] = max(ReliabilityScore(1:numConfigs));
        else
            [~, minIdx] = min(CostperCombo(validCandidates));
            bestID = validCandidates(minIdx);
        end

        SummaryData = struct();
        SummaryData.BestID = bestID;
        SummaryData.CostperCombo = CostperCombo(1:numConfigs);
        SummaryData.ReliabilityScore = ReliabilityScore(1:numConfigs);
        SummaryData.SeparationHistory = separation(1:numConfigs); 
        SummaryData.NumConfigsRun = numConfigs;
        SummaryData.Metadata = SimResults.Metadata; 
        
        save('SimData_Summary_Latest.mat', 'SummaryData');
        
        for i = batchStartIdx:numConfigs
            configStore{i} = [];
            costDetailsStore{i} = [];
            killStore{i} = [];
        end
        
        batchStartIdx = numConfigs + 1;
        
        if configsSinceNewBest >= mcSettings.searchPatience && separation(numConfigs) > mcSettings.minSeparation
            fprintf('Monte Carlo Simulation converged!\n');
            break
        end
    end
end