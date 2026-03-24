clear; clc; close all;


%%  USER CONFIGURATION
%  -------------------

% OPTIMIZATION SETTINGS
mcSettings.maxConfigs = 5; % max number of effector configs to test
mcSettings.testsPerConfig = 5; %  trials per configuration
mcSettings.convergenceDelta = 100; % stop if separation > delta
mcSettings.confidenceAlpha = 0.1; % Student's t-test alpha (0.1 = 90% conf)
mcSettings.reliabilityThresh = 90; % minimum reliability (%) to be valid

% MAP SETTINGS
mapConfig.L = 100; % map Length (units)
mapConfig.W = 100; % map Width (units)
mapConfig.pathBankFile = "adversary_paths_bank.mat";

% ASSET LOCATION
assetConfig.location = [30, 60]; % [X, Y] location of the asset

% EFFECTOR SETTINGS
effConfig.numStatic = 3; % number of static point-defense effectors
effConfig.numMobile = 0; % number of mobile interceptor effectors
effConfig.mobileSpeed = 12; % speed of mobile effectors (units/s)
effConfig.range = 20; % interception radius (units)
effConfig.posBankFile = "effector_posn_bank.mat";

% WEAPON SETTINGS
weaponConfig.mode = "KINETIC"; % "LEGACY", "DIRECT_ENERGY", or "KINETIC"
weaponConfig.directEnergyDwellTime = 0.75;
weaponConfig.kineticProjectileSpeed = 30;
weaponConfig.kineticShotsPerVolley = 3;
weaponConfig.kineticHitProbability = 1.0;
weaponConfig.kineticRefireTime = 0.50;
weaponConfig.projectileHitTolerance = 1.0;
weaponConfig.kineticUseFermiModel = true; 

% ADVERSARY SETTINGS
advConfig.count = 3; % number of incoming threats
advConfig.speed = 15; % UAS Speed (units/s)
advConfig.turnRadius = 2; % UAS Turn Radius (units)
advConfig.altitude = 15; % UAS Ingress Altitude (units)
advConfig.planner = 'HybridAStar'; % path planning algorithm ('linear')

% SENSOR SETTINGS
sensConfig.count = 3; % number of sensors
sensConfig.locations = [30,85; 8.35,47.5; 51.65,47.5]; % [X1,Y1; Xn,Yn] location of sensors
sensConfig.range = 20; % detection radius (units)
sensConfig.params = struct('d50', sensConfig.range, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

% COST FUNCTION SETTINGS
costConfig.effector = 100; % cost per effector used
costConfig.asset = 2000; % cost if asset is destroyed
costConfig.leak = 250; % cost per adversary not intercepted

% SIMULATION ENGINE SETTINGS
simConfig.parallel = false; % run parallelized (no animation) or sequential loop
simConfig.numCores = feature("numcores")/2;
simConfig.tps = 20; % time steps per second
simConfig.animateLive = false; % animate? (slows down processing)


%% SIMULATION SETUP
% -----------------

rng('shuffle')

% define map
load big_island_map.mat
mapObj = elevationMap;
mapBounds = [0 mapConfig.L 0 mapConfig.W];
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

% storage arrays
trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.testsPerConfig);
configStore = cell(mcSettings.maxConfigs, 1);      
scenarioStore = cell(mcSettings.maxConfigs, 1);
pathStore = cell(mcSettings.maxConfigs, 1);
costDetailsStore = cell(mcSettings.maxConfigs, 1); 
killStore = cell(mcSettings.maxConfigs, 1);

CostperCombo = nan(mcSettings.maxConfigs, 1);
ReliabilityScore = nan(mcSettings.maxConfigs, 1); 
LCB = nan(mcSettings.maxConfigs, 1); 
UCB = nan(mcSettings.maxConfigs, 1); 
separation = nan(mcSettings.maxConfigs, 1);


% load position and path banks
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

% define asset
asset = struct('location', assetConfig.location);

% define sensor
sensorStructTemplate = struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params); 
sensors = repmat(sensorStructTemplate, sensConfig.count, 1);
for i = 1:sensConfig.count
    if i <= size(sensConfig.locations, 1)
        sensors(i).location = sensConfig.locations(i,:);
    else
        warning('More sensors requested than locations provided. Sensor %d at [0,0].', i);
    end
end

% define effector template
effConfig.totalEffectors = effConfig.numStatic + effConfig.numMobile;
effectorStructTemplate = struct('location', [0,0], 'range', effConfig.range, ...
    'mode', "STATIC", 'speed', 0, 'heading', 0, 'planner', [], ...
    'path', [], 'pathIdx', 1, 'lastPlanTick', -inf, 'lastInterceptPose', [nan nan nan]);

%% MONTE CARLO SIMULATION
% -----------------------

numConfigs = 0;
fprintf('Starting Monte Carlo Simulation...\n');

while numConfigs < mcSettings.maxConfigs
    numConfigs = numConfigs + 1;
    
    % 1. GENERATE EFFECTORS
    randIndices = randperm(size(effector_posns_bank, 1), effConfig.totalEffectors);
    effPos = effector_posns_bank(randIndices, :);
    
    currentEffectors = repmat(effectorStructTemplate, effConfig.totalEffectors, 1);
    for e = 1:effConfig.totalEffectors
        currentEffectors(e).location = effPos(e, :);
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
    runCosts = zeros(mcSettings.testsPerConfig, 1); 
    runStarts = cell(mcSettings.testsPerConfig, 1);
    runPaths = cell(mcSettings.testsPerConfig, 1);
    runFailures = zeros(mcSettings.testsPerConfig, 1);
    runKills = cell(mcSettings.testsPerConfig, 1);
    
    % 3. RUN TESTS
    if simConfig.parallel
        parpool(simConfig.numCores)
        parfor j = 1:mcSettings.testsPerConfig

            rng(trialSeeds(numConfigs, j), 'twister');

            uasArray = UAS.empty(0, advConfig.count);
            pathIdxs = zeros(advConfig.count, 1);
            starts = zeros(advConfig.count, 3);
            
            for k = 1:advConfig.count
                pIdx = randi(length(adversary_paths_bank));
                pathIdxs(k) = pIdx;
                path = adversary_paths_bank{1,1,pIdx};
                
                startX = path(1, 1);
                startY = path(1, 2);
                
                startZ = advConfig.altitude;
                groundElevation = mapObj.getElevation(startX, startY);
                while groundElevation >= startZ
                    startZ = startZ + 1;
                end
                
                starts(k, :) = [startX, startY, startZ];
                
                uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path);
            end
            
            rngState = rng;
            sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'resetGraphics', false, 'costConfig', costConfig, ...
                'weaponMode', weaponConfig.mode, ...
                'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, ...
                'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, ...
                'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                'kineticRefireTime', weaponConfig.kineticRefireTime, ...
                'projectileHitTolerance', weaponConfig.projectileHitTolerance, ...
                'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
            runResults = sim.runSim();

            runRNGStates{j} = rngState;
            runCosts(j) = runResults.cost;
            runStarts{j} = starts;
            runPaths{j} = pathIdxs;

            if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations)
                runKills{j} = runResults.UASkillLocations;
            end
    
            if runResults.cost >= costConfig.asset
                runFailures(j) = 1;
            end
        end
    else
        for j = 1:mcSettings.testsPerConfig

            rng(trialSeeds(numConfigs, j), 'twister');
    
            uasArray = UAS.empty(0, advConfig.count);
            pathIdxs = zeros(advConfig.count, 1);
            starts = zeros(advConfig.count, 3);
            
            for k = 1:advConfig.count
                pIdx = randi(length(adversary_paths_bank));
                pathIdxs(k) = pIdx;
                path = adversary_paths_bank{1,1,pIdx};
                
                startX = path(1, 1);
                startY = path(1, 2);
                
                startZ = advConfig.altitude;
                groundElevation = mapObj.getElevation(startX, startY);
                while groundElevation >= startZ
                    startZ = startZ + 1;
                end
                
                starts(k, :) = [startX, startY, startZ];
                
                uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, startZ, advConfig.turnRadius, "adversary_path", path);
            end
            rngState = rng;
            
            sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', simConfig.animateLive, 'resetGraphics', true, 'costConfig', costConfig, ...
                'weaponMode', weaponConfig.mode, ...
                'directEnergyDwellTime', weaponConfig.directEnergyDwellTime, ...
                'kineticProjectileSpeed', weaponConfig.kineticProjectileSpeed, ...
                'kineticShotsPerVolley', weaponConfig.kineticShotsPerVolley, ...
                'kineticHitProbability', weaponConfig.kineticHitProbability, ...
                'kineticRefireTime', weaponConfig.kineticRefireTime, ...
                'projectileHitTolerance', weaponConfig.projectileHitTolerance, ...
                'kineticUseFermiModel', weaponConfig.kineticUseFermiModel);
            runResults = sim.runSim();
            
            runRNGStates{j} = rngState;
            runCosts(j) = runResults.cost; 
            runStarts{j} = starts;
            runPaths{j} = pathIdxs;
            
            if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations)
                runKills{j} = runResults.UASkillLocations;
            end
    
            if runResults.cost >= costConfig.asset
                runFailures(j) = 1;
            end
        end
    end

    currentConfigKills = vertcat(runKills{:});
    runFailures = sum(runFailures);
    
    % 4. STORE CONFIGURATION RESULTS
    scenarioStore{numConfigs} = runStarts;
    pathStore{numConfigs} = runPaths;
    costDetailsStore{numConfigs} = runCosts;
    CostperCombo(numConfigs) = mean(runCosts);
    killStore{numConfigs} = currentConfigKills;

    % 5. CONVERGENCE
    currentReliability = 100 * (1 - (runFailures / mcSettings.testsPerConfig));
    ReliabilityScore(numConfigs) = currentReliability;
    
    % Student's t-distribution
    if mcSettings.testsPerConfig > 1
        SD = std(runCosts, 0); 
        SE = SD/sqrt(mcSettings.testsPerConfig); 
        tcrit = tinv(1 - mcSettings.confidenceAlpha/2, mcSettings.testsPerConfig - 1);
        LCB(numConfigs) = CostperCombo(numConfigs) - tcrit*SE;
        UCB(numConfigs) = CostperCombo(numConfigs) + tcrit*SE;
    else
        LCB(numConfigs) = CostperCombo(numConfigs);
        UCB(numConfigs) = CostperCombo(numConfigs);
    end
    
    validMask = find(ReliabilityScore(1:numConfigs) >= mcSettings.reliabilityThresh);
    
    if isempty(validMask) || length(validMask) == 1
        separation(numConfigs) = -inf;
        fprintf('Iter %d: No comparison (Reliability: %.1f%%)\n', numConfigs, currentReliability);
    else
        validCosts = CostperCombo(validMask);
        [~, bestIdxInValid] = min(validCosts);
        bestGlobalID = validMask(bestIdxInValid);
        
        rivalsMask = validMask(validMask ~= bestGlobalID);
        separation(numConfigs) = min(LCB(rivalsMask)) - UCB(bestGlobalID);
        fprintf('Iter %d: Valid Best #%d ($%.0f) vs Rival ($%.0f) | Sep: %.2f\n', ...
            numConfigs, bestGlobalID, CostperCombo(bestGlobalID), ...
            min(CostperCombo(rivalsMask)), separation(numConfigs));
    end
    
    if separation(numConfigs) > mcSettings.convergenceDelta
        fprintf('Monte Carlo Simulation converged!\n');
        break
    end
end

% 6. BEST CONFIGURATION
validCandidates = find(ReliabilityScore(1:numConfigs) >= mcSettings.reliabilityThresh);
if isempty(validCandidates)
    fprintf('\nWARNING: Target Reliability NOT Met. Selecting best available.\n');
    [~, bestID] = max(ReliabilityScore(1:numConfigs));
else
    [minCost, idx] = min(CostperCombo(validCandidates));
    bestID = validCandidates(idx);
    fprintf('\nBest Configuration: Iteration %d\n', bestID);
    fprintf('Reliability: %.1f%% | Avg Cost: $%.2f\n', ReliabilityScore(bestID), minCost);
end


%% SAVING DATA
% ------------

SimResults = struct();

% Metadata
SimResults.Metadata = struct(...
    'Timestamp', datestr(now), ...
    'MapBounds', mapBounds, ...
    'NumAdversaries', advConfig.count, ...
    'NumSensors', sensConfig.count, ...
    'NumEffectors', effConfig.totalEffectors, ...
    'CostConfig', costConfig, ...
    'ReliabilityThreshold', mcSettings.reliabilityThresh, ...
    'SensorParams', sensConfig.params, ...
    'EffectorRange', effConfig.range, ...
    'MCSettings', mcSettings, ... 
    'advConfig', advConfig, ...
    'weaponConfig', weaponConfig ...
);

SimResults.MapData = mapObj; 
SimResults.Asset = asset; 
SimResults.Sensors = sensors;

% configuration results
SimResults.Configs = struct();
for i = 1:numConfigs
    SimResults.Configs(i).ID = i;
    SimResults.Configs(i).Effectors = configStore{i};
    SimResults.Configs(i).CostMean = CostperCombo(i);
    SimResults.Configs(i).Reliability = ReliabilityScore(i);
    SimResults.Configs(i).LCB = LCB(i);
    SimResults.Configs(i).UCB = UCB(i);
    SimResults.Configs(i).KillLocations = killStore{i};
    
    % trial settings
    SimResults.Configs(i).Trials = struct();
    for t = 1:mcSettings.testsPerConfig
        SimResults.Configs(i).Trials(t).rngState = runRNGStates{t};
        SimResults.Configs(i).Trials(t).Starts = scenarioStore{i}{t};
        SimResults.Configs(i).Trials(t).Paths = pathStore{i}{t};
        SimResults.Configs(i).Trials(t).Cost = costDetailsStore{i}(t);
    end
end

SimResults.BestConfigID = bestID;
SimResults.NumConfigsRun = numConfigs;

fileName = sprintf('SimData_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fileName, 'SimResults');
fprintf('\nData saved successfully to: %s\n', fileName);