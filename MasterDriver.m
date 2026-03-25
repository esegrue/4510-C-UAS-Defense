clear; clc; close all;

%% 1. EXPERIMENTAL MATRIX

advCountsToTest = 2:3; % 2-5
effCountsToTest = 2:3; % 2-5
weaponMixesToTest = "DirectEnergy"; % "Kinetic" or "Mixed" or "DirectEnergy" (separate with commas)
mobilityToTest = "Static"; % "Static" or "Mixed" or "Mobile (separate with commas)

%% 2. BASE SIMULATION SETTINGS

% MONTE CARLO SETTINGS
mcSettings.maxConfigs = 10; % 10000
mcSettings.batchSize = 3; % 500
mcSettings.searchPatience = 250; % 250
mcSettings.minSeparation = 50; % 100
mcSettings.confidenceAlpha = 0.1; % 0.1
mcSettings.reliabilityThresh = 0; % 60
mcSettings.minTests = 1; % 50    
mcSettings.maxTests = 5; % 500  
mcSettings.testChunkSize = 1; % 50
mcSettings.targetCIWidth = 100; % 100

% COST SETTINGS
costConfig.effector = 100; % 100
costConfig.asset = 2000; % 2000
costConfig.leak = 250; % 250

% ASSET SETTINGS
asset = struct('location', [30, 60]); % [30, 60]

% WEAPON SETTINGS
weaponConfig = struct('directEnergyDwellTime', 0.75, 'kineticProjectileSpeed', 30, ...
    'kineticShotsPerVolley', 3, 'kineticHitProbability', 1.0, 'kineticRefireTime', 0.50, ...
    'projectileHitTolerance', 1.0, 'kineticUseFermiModel', true);

% ADVERSARY SETTINGS
advConfig = struct('speed', 15, 'turnRadius', 2, 'altitude', 15, 'planner', 'HybridAStar');

% SIM SETTINGS
simConfig = struct('parallel', false, 'numCores', feature("numcores")-1, 'tps', 20);

% MAP SETTINGS
load big_island_map.mat; mapObj = elevationMap;
load('effector_posn_bank.mat', 'effector_posns_bank');
load('adversary_paths_bank.mat', 'adversary_paths_bank');

% SENSOR SETTINGS
sensConfig = struct('range', 20, 'params', struct('d50', 20, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2));
sensors = repmat(struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params), 3, 1);
sensors(1).location = [30,85]; sensors(2).location = [8.35,47.5]; sensors(3).location = [51.65,47.5]; % triangle around asset

if simConfig.parallel && isempty(gcp('nocreate')); parpool(simConfig.numCores); end

sessionID = datestr(now, 'yyyymmdd_HHMMSS');

%% 3. BUILD THE SCENARIO GRID & FOLDERS

baseDir = fullfile(pwd, 'Simulation_Results');
if ~exist(baseDir, 'dir'), mkdir(baseDir); end

scenarioList = []; id = 1;
for a = 1:length(advCountsToTest)
    for e = 1:length(effCountsToTest)
        for w = 1:length(weaponMixesToTest)
            for m = 1:length(mobilityToTest)
                scenarioList(id).ScenarioID = id;
                scenarioList(id).advCount = advCountsToTest(a);
                scenarioList(id).effCount = effCountsToTest(e);
                scenarioList(id).weaponType = weaponMixesToTest(w);
                scenarioList(id).mobilityType = mobilityToTest(m);
                scenarioList(id).converged = false;
                
                prefix = sprintf('Adv%d_Eff%d_%s_%s', advCountsToTest(a), effCountsToTest(e), mobilityToTest(m), weaponMixesToTest(w));
                scenarioList(id).savePrefix = prefix;
                
                folderPath = fullfile(baseDir, prefix);
                if ~exist(folderPath, 'dir'), mkdir(folderPath); end
                scenarioList(id).folderPath = folderPath;
                
                id = id + 1;
            end
        end
    end
end
numScenarios = length(scenarioList);
fprintf('Generated %d unique scenarios. Starting Round-Robin Sweep...\n', numScenarios);

%% 4. STORAGE PREALLOCATION

CostperCombo = nan(numScenarios, mcSettings.maxConfigs);
ReliabilityScore = nan(numScenarios, mcSettings.maxConfigs); 
LCB = nan(numScenarios, mcSettings.maxConfigs); 
UCB = nan(numScenarios, mcSettings.maxConfigs); 
separation = nan(numScenarios, mcSettings.maxConfigs);
configsSinceNewBest = zeros(numScenarios, 1);
bestConfigTracker = zeros(numScenarios, 1);
currentConfigID = zeros(numScenarios, 1);

BatchConfigs = cell(numScenarios, 1); 

fprintf('\nChecking for previous data to resume...\n');
for s = 1:numScenarios
    summaryPath = fullfile(scenarioList(s).folderPath, sprintf('%s_Summary_Latest.mat', scenarioList(s).savePrefix));
    if isfile(summaryPath)
        load(summaryPath, 'SummaryData');
        pastCount = SummaryData.NumConfigsRun;
        currentConfigID(s) = pastCount;
        
        CostperCombo(s, 1:pastCount) = SummaryData.CostperCombo;
        ReliabilityScore(s, 1:pastCount) = SummaryData.ReliabilityScore;
        separation(s, 1:pastCount) = SummaryData.SeparationHistory;
        bestConfigTracker(s) = SummaryData.BestID;
        
        fprintf(' -> Scenario [%s]: Found %d previous configs. Resuming at ID %d.\n', scenarioList(s).savePrefix, pastCount, pastCount + 1);
    else
        fprintf(' -> Scenario [%s]: No previous data. Starting fresh at ID 1.\n', scenarioList(s).savePrefix);
    end
end
fprintf('Starting Round-Robin Sweep...\n\n');

%% 5. MASTER ROUND-ROBIN LOOP

while any(currentConfigID < mcSettings.maxConfigs & ~[scenarioList.converged]')
    
    for s = 1:numScenarios
        if scenarioList(s).converged || currentConfigID(s) >= mcSettings.maxConfigs
            continue; 
        end
        
        currentConfigID(s) = currentConfigID(s) + 1;
        cID = currentConfigID(s);
        
        % 1. Run Evaluator for this Scenario
        currentSeeds = randi([1, 2^31-1], 1, mcSettings.maxTests);
        Res = RunConfig(scenarioList(s), mapObj, asset, sensors, currentSeeds, effector_posns_bank, adversary_paths_bank, mcSettings, costConfig, simConfig, weaponConfig, advConfig);
        
        % 2. Store Matrix Data
        CostperCombo(s, cID) = mean(Res.runCosts);
        currentReliability = 100 * (1 - (Res.runFailures / Res.testsCompleted));
        ReliabilityScore(s, cID) = currentReliability;
        
        if Res.testsCompleted > 1
            currSE = std(Res.runCosts, 0) / sqrt(Res.testsCompleted);
            tcrit = tinv(1 - mcSettings.confidenceAlpha/2, Res.testsCompleted - 1);
            LCB(s, cID) = CostperCombo(s, cID) - tcrit*currSE;
            UCB(s, cID) = CostperCombo(s, cID) + tcrit*currSE;
        else
            LCB(s, cID) = CostperCombo(s, cID); UCB(s, cID) = CostperCombo(s, cID);
        end
        
        % 3. Check Separation & Convergence
        validMask = find(ReliabilityScore(s, 1:cID) >= mcSettings.reliabilityThresh);
        if isempty(validMask) || length(validMask) == 1
            separation(s, cID) = -inf;
        else
            validCosts = CostperCombo(s, validMask); [~, minIdx] = min(validCosts);
            bestGlobalID = validMask(minIdx);
            rivalsMask = validMask(validMask ~= bestGlobalID);
            separation(s, cID) = min(LCB(s, rivalsMask)) - UCB(s, bestGlobalID);
            
            if bestGlobalID ~= bestConfigTracker(s)
                bestConfigTracker(s) = bestGlobalID; configsSinceNewBest(s) = 0;
            else
                configsSinceNewBest(s) = configsSinceNewBest(s) + 1;
            end
            fprintf('[%s] Iter %d: Valid Best #%d ($%.0f) | Sep: %.2f\n', scenarioList(s).savePrefix, cID, bestGlobalID, CostperCombo(s, bestGlobalID), separation(s, cID));
        end
        
        % 4. Package Current Config for Saving
        cfgData = struct('ID', cID, 'Effectors', Res.Effectors, 'CostMean', CostperCombo(s, cID), ...
            'Reliability', ReliabilityScore(s, cID), 'LCB', LCB(s, cID), 'UCB', UCB(s, cID), ...
            'KillLocations', Res.runKills, 'Separation', separation(s, cID), 'TrialsCompleted', Res.testsCompleted);
        
        for t = 1:Res.testsCompleted
            cfgData.Trials(t).Seed = currentSeeds(t);
            cfgData.Trials(t).Starts = Res.runStarts{t};
            cfgData.Trials(t).Paths = Res.runPaths{t};
            cfgData.Trials(t).Cost = Res.runCosts(t);
        end
        
        if isempty(BatchConfigs{s}); BatchConfigs{s} = cfgData; else; BatchConfigs{s}(end+1) = cfgData; end
        
        % 5. Flag Convergence
        if configsSinceNewBest(s) >= mcSettings.searchPatience && separation(s, cID) > mcSettings.minSeparation
            scenarioList(s).converged = true;
            fprintf('\n>>> SCENARIO [%s] CONVERGED AT ITERATION %d <<<\n\n', scenarioList(s).savePrefix, cID);
        end
        
        % 6. Rolling batch saving and RAM clearing
        idxInBatch = mod(cID - 1, mcSettings.batchSize) + 1;
        startIdx = cID - idxInBatch + 1;
        
        % A. CONTINUOUS OVERWRITE (Happens every single iteration for safety)
        Metadata = struct('Timestamp', datestr(now), 'ScenarioConfig', scenarioList(s), ...
            'CostConfig', costConfig, 'MCSettings', mcSettings, 'WeaponConfig', weaponConfig, 'AdvConfig', advConfig);
        
        Configs = BatchConfigs{s};
        
        % Name the file for the full batch span so it safely overwrites the same file
        batchEndIdx = min(startIdx + mcSettings.batchSize - 1, mcSettings.maxConfigs);
        fileName = sprintf('%s_Batch_%dto%d_%s.mat', scenarioList(s).savePrefix, startIdx, batchEndIdx, sessionID);
        fullPath = fullfile(scenarioList(s).folderPath, fileName);
        
        % Save the heavy file
        save(fullPath, 'Metadata', 'mapObj', 'asset', 'sensors', 'Configs', '-v7.3');
        
        % Save the lightweight rolling summary
        SummaryData = struct('BestID', bestConfigTracker(s), 'CostperCombo', CostperCombo(s, 1:cID), ...
            'ReliabilityScore', ReliabilityScore(s, 1:cID), 'SeparationHistory', separation(s, 1:cID), 'NumConfigsRun', cID, 'Metadata', Metadata);
        summaryPath = fullfile(scenarioList(s).folderPath, sprintf('%s_Summary_Latest.mat', scenarioList(s).savePrefix));
        save(summaryPath, 'SummaryData', '-v7.3');
        
        % B. RAM FLUSH (Only happens when the batch is full or the scenario converges)
        if idxInBatch == mcSettings.batchSize || cID == mcSettings.maxConfigs || scenarioList(s).converged
            BatchConfigs{s} = []; % Clear the buffer so the next iteration starts fresh
        end
    end
end

fprintf('All scenarios completed!\n');