clear; clc; close all;

%%  USER CONFIGURATION
%  -------------------
% OPTIMIZATION SETTINGS
mcSettings.maxConfigs = 10000; 
mcSettings.testsPerConfig = 500; 
mcSettings.batchSize = 500; 
mcSettings.convergenceDelta = 10000; 
mcSettings.confidenceAlpha = 0.1; 
mcSettings.reliabilityThresh = 0; 

% MAP SETTINGS
mapConfig.L = 100; 
mapConfig.W = 100; 
mapConfig.pathBankFile = "adversary_paths_bank.mat";

% ASSET LOCATION
assetConfig.location = [30, 60]; 

% EFFECTOR SETTINGS
effConfig.minEffectors = 2; 
effConfig.maxEffectors = 5; 
effConfig.mobilityType = "Static"; 
effConfig.mobileSpeed = 12; 
effConfig.range = 20; 
effConfig.posBankFile = "effector_posn_bank.mat";

% WEAPON SETTINGS (Heterogeneous Loadouts)
weaponConfig.weaponType = "Kinetic"; 
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
simConfig.parallel = true; 
simConfig.numCores = feature("numcores")-1;
if simConfig.parallel && isempty(gcp('nocreate'))
    parpool(simConfig.numCores);
end
simConfig.tps = 20; 
simConfig.animateLive = false; 

%% SIMULATION SETUP
% -----------------
rng('shuffle')
load big_island_map.mat
mapObj = elevationMap;
mapBounds = [0 mapConfig.L 0 mapConfig.W];
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.testsPerConfig);
maxE = effConfig.maxEffectors;

configStore = cell(maxE, mcSettings.maxConfigs);      
scenarioStore = cell(maxE, mcSettings.maxConfigs);
pathStore = cell(maxE, mcSettings.maxConfigs);
costDetailsStore = cell(maxE, mcSettings.maxConfigs); 
killStore = cell(maxE, mcSettings.maxConfigs);
rngStore = cell(maxE, mcSettings.maxConfigs); 

CostperCombo = nan(maxE, mcSettings.maxConfigs);
ReliabilityScore = nan(maxE, mcSettings.maxConfigs); 
LCB = nan(maxE, mcSettings.maxConfigs); 
UCB = nan(maxE, mcSettings.maxConfigs); 
separation = nan(maxE, mcSettings.maxConfigs);

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
        warning('More sensors requested than locations provided.');
    end
end

effectorStructTemplate = struct('location', [0,0], 'range', effConfig.range, ...
    'mode', "STATIC", 'weaponMode', "LEGACY", 'speed', 0, 'heading', 0, 'planner', [], ...
    'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);

%% MONTE CARLO SIMULATION
% -----------------------
numConfigs = 0;
converged = false(maxE, 1);

% --- Initialize our rolling states outside the loop ---
batchTimestamps = cell(maxE, 1); 
BatchConfigs = cell(maxE, 1); 

fprintf('Starting Monte Carlo Simulation...\n');

while numConfigs < mcSettings.maxConfigs && ~all(converged(effConfig.minEffectors:effConfig.maxEffectors))
    numConfigs = numConfigs + 1;
    
    for totEff = effConfig.minEffectors:effConfig.maxEffectors
        
        if converged(totEff); continue; end
        
        if effConfig.mobilityType == "Static"
            effConfig.numStatic = totEff; effConfig.numMobile = 0;
        else
            effConfig.numStatic = 0; effConfig.numMobile = totEff;
        end

        if weaponConfig.weaponType == "Kinetic"
            weaponConfig.numKinetic = totEff; weaponConfig.numDirectEnergy = 0;
        else
            weaponConfig.numKinetic = 0; weaponConfig.numDirectEnergy = totEff;
        end
        weaponConfig.numLegacy = 0;

        effConfig.totalEffectors = effConfig.numStatic + effConfig.numMobile;
        weaponTypes = [repmat("LEGACY", 1, weaponConfig.numLegacy), ...
                       repmat("DIRECT_ENERGY", 1, weaponConfig.numDirectEnergy), ...
                       repmat("KINETIC", 1, weaponConfig.numKinetic)];
                       
        if length(weaponTypes) ~= effConfig.totalEffectors
            error('The sum of weapon types must exactly equal effConfig.totalEffectors.');
        end
        
        % 1. GENERATE EFFECTORS
        randIndices = randperm(size(effector_posns_bank, 1), effConfig.totalEffectors);
        effPos = effector_posns_bank(randIndices, :);
        currentEffectors = repmat(effectorStructTemplate, effConfig.totalEffectors, 1);
        for e = 1:effConfig.totalEffectors
            currentEffectors(e).location = effPos(e, :);
            currentEffectors(e).weaponMode = weaponTypes(e); 
            if e <= effConfig.numStatic
                currentEffectors(e).mode = "STATIC"; currentEffectors(e).speed = 0;
            else
                currentEffectors(e).mode = "MOBILE"; currentEffectors(e).speed = effConfig.mobileSpeed;
                dir2D = asset.location - currentEffectors(e).location;
                if norm(dir2D) > 0
                    currentEffectors(e).heading = atan2(dir2D(2), dir2D(1));
                else
                    currentEffectors(e).heading = 0;
                end
            end
        end
        configStore{totEff, numConfigs} = currentEffectors;
        
        % 2. INITIALIZE STORAGE
        runCosts = zeros(mcSettings.testsPerConfig, 1); 
        runStarts = cell(mcSettings.testsPerConfig, 1);
        runPaths = cell(mcSettings.testsPerConfig, 1);
        runFailures = zeros(mcSettings.testsPerConfig, 1);
        runKills = cell(mcSettings.testsPerConfig, 1);
        runRNGStates = cell(mcSettings.testsPerConfig, 1);
        
        % 3. RUN TESTS
        if simConfig.parallel
            parfor j = 1:mcSettings.testsPerConfig
                rng(trialSeeds(numConfigs, j), 'twister');
                uasArray = UAS.empty(0, advConfig.count); pathIdxs = zeros(advConfig.count, 1); starts = zeros(advConfig.count, 3);
                for k = 1:advConfig.count
                    pIdx = randi(length(adversary_paths_bank)); pathIdxs(k) = pIdx; path = adversary_paths_bank{1,1,pIdx};
                    startX = path(1, 1); startY = path(1, 2); startZ = advConfig.altitude;
                    groundElevation = mapObj.getElevation(startX, startY);
                    while groundElevation >= startZ; startZ = startZ + 1; end
                    starts(k, :) = [startX, startY, startZ];
                    uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path);
                end
                rngState = rng;
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'resetGraphics', false, 'costConfig', costConfig, ...
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                runRNGStates{j} = rngState; runCosts(j) = runResults.cost; runStarts{j} = starts; runPaths{j} = pathIdxs;
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); runKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; runFailures(j) = 1; end
            end
        else
            for j = 1:mcSettings.testsPerConfig
                rng(trialSeeds(numConfigs, j), 'twister');
                uasArray = UAS.empty(0, advConfig.count); pathIdxs = zeros(advConfig.count, 1); starts = zeros(advConfig.count, 3);
                for k = 1:advConfig.count
                    pIdx = randi(length(adversary_paths_bank)); pathIdxs(k) = pIdx; path = adversary_paths_bank{1,1,pIdx};
                    startX = path(1, 1); startY = path(1, 2); startZ = advConfig.altitude;
                    groundElevation = mapObj.getElevation(startX, startY);
                    while groundElevation >= startZ; startZ = startZ + 1; end
                    starts(k, :) = [startX, startY, startZ];
                    uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path);
                end
                rngState = rng;
                sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', simConfig.animateLive, 'resetGraphics', true, 'costConfig', costConfig, ...
                    'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, 'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                    'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, 'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                    'kineticRefireTime', weaponConfig.kineticRefireTime, 'projectileHitTolerance', weaponConfig.projectileHitTolerance, 'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
                runResults = sim.runSim();
                runRNGStates{j} = rngState; runCosts(j) = runResults.cost; runStarts{j} = starts; runPaths{j} = pathIdxs;
                if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations); runKills{j} = runResults.UASkillLocations; end
                if runResults.cost >= costConfig.asset; runFailures(j) = 1; end
            end
        end
        currentConfigKills = vertcat(runKills{:}); runFailures = sum(runFailures);
        
        % 4. STORE CONFIGURATION RESULTS
        scenarioStore{totEff, numConfigs} = runStarts; pathStore{totEff, numConfigs} = runPaths; costDetailsStore{totEff, numConfigs} = runCosts;
        CostperCombo(totEff, numConfigs) = mean(runCosts); killStore{totEff, numConfigs} = currentConfigKills; rngStore{totEff, numConfigs} = runRNGStates; 
        
        % 5. CONVERGENCE
        currentReliability = 100 * (1 - (runFailures / mcSettings.testsPerConfig)); ReliabilityScore(totEff, numConfigs) = currentReliability;
        if mcSettings.testsPerConfig > 1
            SD = std(runCosts, 0); SE = SD/sqrt(mcSettings.testsPerConfig); tcrit = tinv(1 - mcSettings.confidenceAlpha/2, mcSettings.testsPerConfig - 1);
            LCB(totEff, numConfigs) = CostperCombo(totEff, numConfigs) - tcrit*SE; UCB(totEff, numConfigs) = CostperCombo(totEff, numConfigs) + tcrit*SE;
        else
            LCB(totEff, numConfigs) = CostperCombo(totEff, numConfigs); UCB(totEff, numConfigs) = CostperCombo(totEff, numConfigs);
        end
        validMask = find(ReliabilityScore(totEff, 1:numConfigs) >= mcSettings.reliabilityThresh);
        if isempty(validMask) || length(validMask) == 1
            separation(totEff, numConfigs) = -inf;
            fprintf('[TotEff: %d] Iter %d: No comparison\n', totEff, numConfigs);
        else
            validCosts = CostperCombo(totEff, validMask); [~, bestIdxInValid] = min(validCosts); bestGlobalID = validMask(bestIdxInValid);
            rivalsMask = validMask(validMask ~= bestGlobalID); separation(totEff, numConfigs) = min(LCB(totEff, rivalsMask)) - UCB(totEff, bestGlobalID);
            fprintf('[TotEff: %d] Iter %d: Valid Best #%d ($%.0f) vs Rival ($%.0f) | Sep: %.2f\n', ...
                totEff, numConfigs, bestGlobalID, CostperCombo(totEff, bestGlobalID), min(CostperCombo(totEff, rivalsMask)), separation(totEff, numConfigs));
        end
        
        % ---------------------------------------------------------
        % 6. INCREMENTAL BATCH SAVING, ROLLING SUMMARY & RAM FLUSH
        % ---------------------------------------------------------
        
        idxInBatch = mod(numConfigs - 1, mcSettings.batchSize) + 1;
        startIdx = numConfigs - idxInBatch + 1;
        endIdx = min(startIdx + mcSettings.batchSize - 1, mcSettings.maxConfigs);

        folder = '\\coeit.osu.edu\home\s\segrue.3\Desktop\C-UAS_Defense-2.30\C-UAS_Defense-2.30\Run1_NumEffectors';
        if ~exist(folder, 'dir')
            mkdir(folder); % Ensure directory exists
        end
        savePrefix = sprintf('Tot%d_%s_%s', totEff, effConfig.mobilityType, weaponConfig.weaponType);
        
        Metadata = struct('Timestamp', datestr(now), 'MapBounds', mapBounds, ...
            'NumAdversaries', advConfig.count, 'NumSensors', sensConfig.count, ...
            'NumEffectors', totEff, 'CostConfig', costConfig, ...
            'ReliabilityThreshold', mcSettings.reliabilityThresh, ...
            'SensorParams', sensConfig.params, 'EffectorRange', effConfig.range, ...
            'MCSettings', mcSettings, 'advConfig', advConfig, 'weaponConfig', weaponConfig);
        
        % Lock in the timestamp for the file name if starting a new batch
        if idxInBatch == 1
            batchTimestamps{totEff} = datestr(now, 'yyyymmdd_HHMMSS');
        end
        
        fileName = sprintf('%s_Batch_%dto%d_%s.mat', savePrefix, startIdx, endIdx, batchTimestamps{totEff});
        fullPath = fullfile(folder, fileName);

        % Package the current configuration
        currentConfigData = struct();
        currentConfigData.ID = numConfigs;
        currentConfigData.Effectors = configStore{totEff, numConfigs};
        currentConfigData.CostMean = CostperCombo(totEff, numConfigs);
        currentConfigData.Reliability = ReliabilityScore(totEff, numConfigs);
        currentConfigData.LCB = LCB(totEff, numConfigs);
        currentConfigData.UCB = UCB(totEff, numConfigs);
        currentConfigData.KillLocations = killStore{totEff, numConfigs};
        currentConfigData.Separation = separation(totEff, numConfigs);
        
        currentConfigData.Trials = struct();
        for t = 1:mcSettings.testsPerConfig
            currentConfigData.Trials(t).rngState = rngStore{totEff, numConfigs}{t};
            currentConfigData.Trials(t).Starts = scenarioStore{totEff, numConfigs}{t};
            currentConfigData.Trials(t).Paths = pathStore{totEff, numConfigs}{t};
            currentConfigData.Trials(t).Cost = costDetailsStore{totEff, numConfigs}(t);
        end
        
        % Append the current configuration to the Rolling RAM Buffer
        if isempty(BatchConfigs{totEff})
            BatchConfigs{totEff} = currentConfigData;
        else
            BatchConfigs{totEff}(end+1) = currentConfigData;
        end
        
        % --- THE MAGIC: Full Safe Overwrite ---
        % By saving the whole file cleanly, we avoid network drive indexing errors 
        % and HDF5 file bloat. It runs incredibly fast and guarantees integrity.
        Configs = BatchConfigs{totEff}; 
        MapData = mapObj; AssetData = asset; SensorData = sensors;
        save(fullPath, 'Metadata', 'MapData', 'AssetData', 'SensorData', 'Configs', '-v7.3');
        
        % Update the rolling summary
        validCandidates = find(ReliabilityScore(totEff, 1:numConfigs) >= mcSettings.reliabilityThresh);
        if isempty(validCandidates)
            [~, bestID] = max(ReliabilityScore(totEff, 1:numConfigs));
        else
            [~, minIdx] = min(CostperCombo(totEff, validCandidates));
            bestID = validCandidates(minIdx);
        end
        
        SummaryData = struct();
        SummaryData.BestID = bestID;
        SummaryData.CostperCombo = CostperCombo(totEff, 1:numConfigs);
        SummaryData.ReliabilityScore = ReliabilityScore(totEff, 1:numConfigs);
        SummaryData.SeparationHistory = separation(totEff, 1:numConfigs);
        SummaryData.NumConfigsRun = numConfigs;
        SummaryData.Metadata = Metadata;
        
        summaryFileName = sprintf('%s_Summary_Latest.mat', savePrefix);
        summaryFullPath = fullfile(folder, summaryFileName);
        save(summaryFullPath, 'SummaryData', '-v7.3');
        
        % RAM Flush: Instantly clear out cell arrays to keep memory clean
        configStore{totEff, numConfigs} = [];
        scenarioStore{totEff, numConfigs} = [];
        pathStore{totEff, numConfigs} = [];
        costDetailsStore{totEff, numConfigs} = [];
        killStore{totEff, numConfigs} = [];
        rngStore{totEff, numConfigs} = [];
        
        % If the batch is completely full, dump the RAM Buffer so the next file starts fresh
        if idxInBatch == mcSettings.batchSize || numConfigs == mcSettings.maxConfigs || separation(totEff, numConfigs) > mcSettings.convergenceDelta
            BatchConfigs{totEff} = [];
        end
        
        % Check if we met convergence
        if separation(totEff, numConfigs) > mcSettings.convergenceDelta
            converged(totEff) = true;
            fprintf('[TotEff: %d] Converged at config %d!\n', totEff, numConfigs);
        end
        
    end
end
