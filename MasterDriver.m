clear; clc; close all;

%% 1. EXPERIMENTAL MATRIX

advCountsToTest = 2:2:10; % 3 & 4
effCountsToTest = 8:2:10; % 2-5
weaponToTest = ["Mixed"]; % "Kinetic" or "Mixed" or "DirectEnergy" (separate with commas)
mobilityToTest = ["Mobile"]; % "Static" or "Mixed" or "Mobile (separate with commas)

%% 2. BASE SIMULATION SETTINGS

% MONTE CARLO SETTINGS
mcSettings.maxConfigs = 1000000; % 1000000
mcSettings.batchSize = 15; % 50
mcSettings.searchPatience = 250; % 250
mcSettings.minSeparation = 100; % 100
mcSettings.confidenceAlpha = 0.1; % 0.1
mcSettings.reliabilityThresh = 0; % 50
mcSettings.minTests = 100; % 50    
mcSettings.maxTests = 450; % 750  
mcSettings.testChunkSize = 50; % 50
mcSettings.targetCIWidth = 125; % 100

% ASSET SETTINGS
asset = struct('location', [30, 60]); % [30, 60]

% COST SETTINGS
costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250, ...
    'maxRiskPenalty', 500, 'riskDecayRate', 0.1);

% WEAPON / EFFECTOR SETTINGS
weaponConfig = struct('directEnergyDwellTime', 0.5, 'kineticProjectileSpeed', 75, ...
    'kineticShotsPerVolley', 2, 'kineticHitProbability', 0.4, 'kineticRefireTime', 0.5, ...
    'projectileHitTolerance', 1.0, 'kineticUseFermiModel', true, ...
    'effectorRange', 20, 'mobileSpeed', 12); 

% ADVERSARY SETTINGS
advConfig = struct('speed', 15, 'turnRadius', 2, 'altitude', 15, ...
    'planner', 'HybridAStar', 'searchRange', 20);

% SIM SETTINGS
c = parcluster; 
simConfig = struct('parallel', true, 'numCores', c.NumWorkers, 'tps', 20);

% MAP SETTINGS
load big_island_map.mat; mapObj = elevationMap;
load('effector_posns_bank.mat', 'effector_posns_bank');
load('adversary_paths_bank.mat', 'adversary_paths_bank');
mapBounds = [0, 100, 0, 100];

% SENSOR SETTINGS
sensConfig = struct('range', 20, 'params', struct('d50', 20, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2));
sensors = repmat(struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params), 3, 1);
sensors(1).location = asset.location; sensors(2).location = [72, 55]; sensors(3).location = [20, 37]; % triangle around asset [30,85],[8.35,47.5],[51.65,47.5]

if simConfig.parallel && isempty(gcp('nocreate')); parpool(simConfig.numCores); end

sessionID = datestr(now, 'yyyymmdd_HHMMSS');

%% 3. BUILD THE SCENARIO GRID & FOLDERS

baseDir = fullfile(pwd, 'Simulation_Results_Mobile_Mixed');
if ~exist(baseDir, 'dir'), mkdir(baseDir); end

scenarioList = []; id = 1;
for a = 1:length(advCountsToTest)
    for e = 1:length(effCountsToTest)
        for w = 1:length(weaponToTest)
            for m = 1:length(mobilityToTest)
                scenarioList(id).ScenarioID = id;
                scenarioList(id).advCount = advCountsToTest(a);
                scenarioList(id).effCount = effCountsToTest(e);
                scenarioList(id).weaponType = weaponToTest(w);
                scenarioList(id).mobilityType = mobilityToTest(m);
                scenarioList(id).converged = false;
                
                prefix = sprintf('Adv%d_Eff%d_%s_%s', advCountsToTest(a), effCountsToTest(e), mobilityToTest(m), weaponToTest(w));
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

%% 4. STORAGE PREALLOCATION & ENVIRONMENT SETUP

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
    envPath = fullfile(scenarioList(s).folderPath, sprintf('%s_Environment.mat', scenarioList(s).savePrefix));
    
    if isfile(summaryPath)
        load(summaryPath, 'SummaryData');
        pastCount = SummaryData.NumConfigsRun;
        currentConfigID(s) = pastCount;
        
        CostperCombo(s, 1:pastCount) = SummaryData.CostperCombo;
        ReliabilityScore(s, 1:pastCount) = SummaryData.ReliabilityScore;
        separation(s, 1:pastCount) = SummaryData.SeparationHistory;
        bestConfigTracker(s) = SummaryData.BestID;
        
        % --- NEW AUTOMATED RECOVERY BLOCK ---
        if isfield(SummaryData, 'LCB') && isfield(SummaryData, 'UCB')
            % Standard load if the new save format is already in place
            LCB(s, 1:pastCount) = SummaryData.LCB;
            UCB(s, 1:pastCount) = SummaryData.UCB;
        else
            % Legacy resume: Reconstruct LCB/UCB from Batch files
            fprintf('    > Legacy summary detected for [%s]. Reconstructing LCB/UCB...\n', scenarioList(s).savePrefix);
            batchFiles = dir(fullfile(scenarioList(s).folderPath, '*_Batch_*.mat'));
            for b = 1:length(batchFiles)
                batchData = load(fullfile(scenarioList(s).folderPath, batchFiles(b).name));
                for c = 1:length(batchData.Configs)
                    idx = batchData.Configs(c).ID;
                    if idx <= pastCount
                        LCB(s, idx) = batchData.Configs(c).LCB;
                        UCB(s, idx) = batchData.Configs(c).UCB;
                    end
                end
            end
            
            % Patch the summary file on disk so it loads instantly next time
            SummaryData.LCB = LCB(s, 1:pastCount);
            SummaryData.UCB = UCB(s, 1:pastCount);
            save(summaryPath, 'SummaryData');
        end
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
            fprintf('[%s] Iter %d: Gathering initial data (Cost: $%.0f, Rel: %.1f%%)\n', ...
                scenarioList(s).savePrefix, cID, CostperCombo(s, cID), ReliabilityScore(s, cID));
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
        
        % 4. Package Current Config for RAM Buffer
        cfgData = struct('ID', cID, 'Effectors', Res.Effectors, 'CostMean', CostperCombo(s, cID), ...
            'Reliability', ReliabilityScore(s, cID), 'LCB', LCB(s, cID), 'UCB', UCB(s, cID), ...
            'KillLocations', Res.runKills, 'Separation', separation(s, cID), 'TrialsCompleted', Res.testsCompleted);
        
        for t = 1:Res.testsCompleted
            cfgData.Trials(t).Seed = currentSeeds(t);
            cfgData.Trials(t).Cost = Res.runCosts(t);
        end
        
        if isempty(BatchConfigs{s}); BatchConfigs{s} = cfgData; else; BatchConfigs{s}(end+1) = cfgData; end
        
        % 5. Flag Convergence
        if configsSinceNewBest(s) >= mcSettings.searchPatience && separation(s, cID) > mcSettings.minSeparation
            scenarioList(s).converged = true;
            fprintf('\n>>> SCENARIO [%s] CONVERGED AT ITERATION %d <<<\n\n', scenarioList(s).savePrefix, cID);
        end
        
        % 6. Throttled Rolling Save (Disk I/O optimized)
        idxInBatch = mod(cID - 1, mcSettings.batchSize) + 1;
        startIdx = cID - idxInBatch + 1;
        
        % Only write to disk when the batch is full, scenario converges, or max limit is reached
        if idxInBatch == mcSettings.batchSize || cID == mcSettings.maxConfigs || scenarioList(s).converged
            
            % A. SAVE THE CHUNK (Dynamic Data Only, Standard .mat format for speed)
            Configs = BatchConfigs{s};
            batchEndIdx = cID;
            fileName = sprintf('%s_Batch_%dto%d_%s.mat', scenarioList(s).savePrefix, startIdx, batchEndIdx, sessionID);
            fullPath = fullfile(scenarioList(s).folderPath, fileName);
            save(fullPath, 'Configs'); 
            
            % B. OVERWRITE THE SUMMARY (Lightweight arrays only)
            SummaryData = struct('BestID', bestConfigTracker(s), 'CostperCombo', CostperCombo(s, 1:cID), ...
                'ReliabilityScore', ReliabilityScore(s, 1:cID), 'SeparationHistory', separation(s, 1:cID), ...
                'LCB', LCB(s, 1:cID), 'UCB', UCB(s, 1:cID), ...
                'NumConfigsRun', cID);
            summaryPath = fullfile(scenarioList(s).folderPath, sprintf('%s_Summary_Latest.mat', scenarioList(s).savePrefix));
            save(summaryPath, 'SummaryData');
            
            % C. RAM FLUSH
            BatchConfigs{s} = []; 
        end
    end
end
fprintf('All scenarios completed!\n');