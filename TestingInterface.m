clear; clc; close all;


%%  USER CONFIGURATION
%  -------------------

% OPTIMIZATION SETTINGS
mcSettings.maxConfigs = 200; % max number of effector configs to test
mcSettings.testsPerConfig = 1; %  trials per configuration
mcSettings.convergenceDelta = 100; % stop if separation > delta
mcSettings.confidenceAlpha = 0.1; % Student's t-test alpha (0.1 = 90% conf)
mcSettings.reliabilityThresh = 90; % minimum reliability (%) to be valid

% MAP SETTINGS
mapConfig.L = 100; % map Length (units)
mapConfig.W = 100; % map Width (units)
mapConfig.terrainType = 'Hills'; % terrain generation type ('Flat')
mapConfig.terrainMag = 30; % terrain height magnitude

% ASSET LOCATION
assetConfig.location = [30, 60]; % [X, Y] location of the asset

% EFFECTOR SETTINGS
effConfig.numEffectors = 3; % number of effectors to place
effConfig.range = 20; % interception radius (units)
effConfig.posBankFile = "effector_posn_bank.mat"; % source file for positions

% ADVERSARY SETTINGS
advConfig.count = 20; % number of incoming threats
advConfig.speed = 15; % UAS Speed (units/s)
advConfig.turnRadius = 25; % UAS Turn Radius (units)
advConfig.planner = 'HybridAStar'; % path planning algorithm ('linear')

% SENSOR SETTINGS
sensConfig.count = 3; % number of sensors
sensConfig.locations = [30,85; 8.35,47.5; 51.65,47.5]; % [X1,Y1; Xn,Yn] location of sensors
sensConfig.range = 25; % detection radius (units)
sensConfig.params = struct('d50', 25, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

% COST FUNCTION SETTINGS
costConfig.effector = 100; % cost per effector used
costConfig.asset = 2000; % cost if asset is destroyed
costConfig.leak = 250; % cost per adversary not intercepted

% SIMULATION ENGINE SETTINGS
simConfig.tps = 20; % time steps per second
simConfig.animateLive = false; % animate? (slows down processing)


%% SIMULATION SETUP
% -----------------

% define map
mapBounds = [0 mapConfig.L 0 mapConfig.W];
mapObj = map(mapConfig.L, mapConfig.W, 1); 
mapObj.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag); 
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

% storage arrays
configStore = cell(mcSettings.maxConfigs, 1);      
scenarioStore = cell(mcSettings.maxConfigs, 1);    
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
effectorStructTemplate = struct('location', [0,0], 'range', effConfig.range);


%% MONTE CARLO SIMULATION
% -----------------------

numConfigs = 0;
fprintf('Starting Monte Carlo Simulation...\n');

while numConfigs < mcSettings.maxConfigs
    numConfigs = numConfigs + 1;
    
    % 1. GENERATE EFFECTORS
    randIndices = randi(size(effector_posns_bank, 1), [1, effConfig.numEffectors]);
    effPos = effector_posns_bank(randIndices, :);
    
    currentEffectors = repmat(effectorStructTemplate, effConfig.numEffectors, 1);
    for e = 1:effConfig.numEffectors
        currentEffectors(e).location = effPos(e, :);
    end
    configStore{numConfigs} = currentEffectors;

    % 2. INITIALIZE STORAGE
    runCosts = zeros(mcSettings.testsPerConfig, 1); 
    runStarts = cell(mcSettings.testsPerConfig, 1);
    runFailures = 0;
    currentConfigKills = [];
    
    % 3. RUN TESTS
    for j = 1:mcSettings.testsPerConfig
        starts = ingressPosns(xlims, ylims, advConfig.count);
        
        runSeed = numConfigs*1000 + j;
        rng(runSeed);

        uasArray = UAS.empty(0, advConfig.count);
        for k = 1:advConfig.count
            uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, ...
                              advConfig.planner, advConfig.turnRadius);
        end
        
        sim = simulator(mapObj, uasArray, currentEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', simConfig.animateLive, 'nfzs', polyshape.empty, 'resetGraphics', true, 'costConfig', costConfig);
        runResults = sim.runSim();
        
        runCosts(j) = runResults.cost; 
        runStarts{j} = starts;
        
        if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations)
            currentConfigKills = [currentConfigKills; runResults.UASkillLocations];
        end

        if runResults.cost >= costConfig.asset
            runFailures = runFailures + 1;
        end
    end
    
    % 4. STORE CONFIGURATION RESULTS
    scenarioStore{numConfigs} = runStarts;
    costDetailsStore{numConfigs} = runCosts;
    CostperCombo(numConfigs) = mean(runCosts);
    killStore{numConfigs} = currentConfigKills;

    % 5. CONVERGENCE
    currentReliability = 100 * (1 - (runFailures / mcSettings.testsPerConfig));
    ReliabilityScore(numConfigs) = currentReliability;
    
    % Student's t-distribution
    if mcSettings.testsPerConfig > 1
        SD = std(runCosts, 1); 
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
    'NumEffectors', effConfig.numEffectors, ...
    'CostConfig', costConfig, ...
    'ReliabilityThreshold', mcSettings.reliabilityThresh, ...
    'SensorParams', sensConfig.params, ...
    'EffectorRange', effConfig.range, ...
    'MCSettings', mcSettings ... 
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
        SimResults.Configs(i).Trials(t).Seed = i*1000 + t;
        SimResults.Configs(i).Trials(t).Starts = scenarioStore{i}{t};
        SimResults.Configs(i).Trials(t).Cost = costDetailsStore{i}(t);
    end
end

SimResults.BestConfigID = bestID;
SimResults.NumConfigsRun = numConfigs;

fileName = sprintf('SimData_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fileName, 'SimResults');
fprintf('\nData saved successfully to: %s\n', fileName);