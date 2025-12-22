clear; clc; close all;

% SIMULATION PARAMETERS
% defining map
mapL = 100; mapW = 100;
mapObj = map(mapL, mapW, 1); 
mapObj.generateTerrain('Hills', 30); 
xlims = [0 mapL]; ylims = [0 mapW];

% simulation settings
numAssets = 2; numSensors = 2; numEffectors = 2; numAdversaries = 3;

% defining asset
assetStructTemplate = struct('location', [0,0]);
assets = repmat(assetStructTemplate, numAssets, 1);
assets(1).location = [30, 60]; assets(2).location = [60, 45];

% defining sensors
sensorRange = 25; 

% TUNING: High-speed scanner (5Hz)
% - scanRate: 0.2s (checks 5 times/sec)
% - pings: 3 (requires 0.6s continuous contact to lock)
% - k: 10 (probability drops off gradually at the edge)
params = struct('d50', sensorRange, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

sensorStructTemplate = struct('location', [0,0], 'range', sensorRange, 'params', params); 
sensors = repmat(sensorStructTemplate, numSensors, 1);
sensorLocs = [assets(1).location; assets(2).location];
for i = 1:numSensors
    sensors(i).location = sensorLocs(i,:);
end

% initializing effectors
effectorRange = 25; 
effectorStructTemplate = struct('location', [0,0], 'range', effectorRange);

% define AOR - what do we want to do with this?
AORo = (assets(1).location + assets(2).location)/2; AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], ...
                [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);

% defining cost function
costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250);
reliabilityThreshold = 90; 

% MONTE CARLO SETTINGS
max_configs = 100;
num_tests = 20;
delta = 10; 

configStore = cell(max_configs, 1);      
scenarioStore = cell(max_configs, 1);    
costDetailsStore = cell(max_configs, 1); 

CostperCombo = nan(max_configs, 1);
ReliabilityScore = nan(max_configs, 1); 
LCB = nan(max_configs, 1); UCB = nan(max_configs, 1); 
separation = nan(max_configs, 1);

% CONFIG LOOP
num_configs = 0;
while num_configs < max_configs
    num_configs = num_configs + 1;
    
    % generating effectors
    effPos = effectorPosnsGenerator(xlims, ylims, numEffectors); 
    currentEffectors = repmat(effectorStructTemplate, numEffectors, 1);
    for e = 1:numEffectors
        currentEffectors(e).location = effPos(e, :);
    end
    configStore{num_configs} = currentEffectors;

    % initialize scenario storage
    runCosts = zeros(num_tests, 1); 
    runStarts = cell(num_tests, 1);
    runFailures = 0;
    
    % SCENARIO LOOP
    parfor j = 1:num_tests
        % generating adversary
        starts = ingressPosns(xlims, ylims, numAdversaries);
        uasArray = UAS.empty(0, numAdversaries);
        for k = 1:numAdversaries
            uasArray(k) = UAS(15, starts(k,:), assets(1).location, 'Linear', 25);
        end
        
        % simulating
        sim = simulator(mapObj, AOR, uasArray, currentEffectors, sensors, assets, 'tps', 20, 'animate', false, 'nfzs', polyshape.empty, 'resetGraphics', true, 'costConfig', costConfig);
        runResults = sim.runSim();
        runCosts(j) = runResults.cost; 
        runStarts{j} = starts;
        
        if runResults.cost >= costConfig.asset
            runFailures = runFailures + 1;
        end
    end
    
    % storing config results
    scenarioStore{num_configs} = runStarts;
    costDetailsStore{num_configs} = runCosts;
    CostperCombo(num_configs) = mean(runCosts);

    % CONDITIONAL CONVERGENCE
    % calculating config reliability
    currentReliability = 100 * (1 - (runFailures / num_tests));
    ReliabilityScore(num_configs) = currentReliability;
    
    % Student's t-distribution
    SD = std(runCosts, 1); 
    SE = SD/sqrt(num_tests); 
    alpha = 0.1;
    tcrit = tinv(1 - alpha/2, num_tests - 1);
    LCB(num_configs) = CostperCombo(num_configs) - tcrit*SE;
    UCB(num_configs) = CostperCombo(num_configs) + tcrit*SE;
    
    % filtering unreliable configs
    validMask = find(ReliabilityScore(1:num_configs) >= reliabilityThreshold);
    
    % comparing configs
    if isempty(validMask) || length(validMask) == 1
        separation(num_configs) = -inf;
        fprintf('Iter %d: No comparison (Reliability: %.1f%%)\n', num_configs, currentReliability);
    else
        validCosts = CostperCombo(validMask);
        [~, bestIdxInValid] = min(validCosts);
        bestGlobalID = validMask(bestIdxInValid);
        
        rivalsMask = validMask(validMask ~= bestGlobalID);
        separation(num_configs) = min(LCB(rivalsMask)) - UCB(bestGlobalID);
        fprintf('Iter %d: Valid Best #%d ($%.0f) vs Rival ($%.0f) | Sep: %.2f\n', num_configs, bestGlobalID, CostperCombo(bestGlobalID), min(CostperCombo(rivalsMask)), separation(num_configs));
    end
    
    if separation(num_configs) > delta
        fprintf('Monte Carlo Simulation converged!\n');
        break
    end
end

% CONFIG SELECTION
validCandidates = find(ReliabilityScore(1:num_configs) >= reliabilityThreshold);
if isempty(validCandidates)
    fprintf('\nWARNING: Target Reliability NOT Met. Selecting best available.\n');
    [~, bestID] = max(ReliabilityScore(1:num_configs));
else
    [minCost, idx] = min(CostperCombo(validCandidates));
    bestID = validCandidates(idx);
    fprintf('\nBest Configuration: Iteration %d\n', bestID);
    fprintf('Reliability: %.1f%% | Avg Cost: $%.2f\n', ReliabilityScore(bestID), minCost);
end

% VISUALIZING
% plotting histogram
figure('Name', 'Reliability Analysis', 'Position', [100 100 600 400]);
histogram(costDetailsStore{bestID}, 'BinMethod', 'auto', 'FaceColor', 'g');
xline(costConfig.asset, 'r--', 'LineWidth', 2, 'Label', 'Asset Loss');
xline(costConfig.leak, 'k--', 'LineWidth', 1.5, 'Label', 'Leak');
xline(costConfig.effector * numAdversaries, 'b--', 'LineWidth', 1.5, 'Label', 'Ideal');
title(sprintf('Best Config (#%d) Cost Distribution', bestID));
xlabel('Scenario Cost ($)'); ylabel('Frequency'); grid on;

% plotting winning config
figure('Name', 'Winning Configuration', 'Position', [100 100 600 400]);
fprintf('\nReplaying Winning Configuration...\n');

scenariosToReplay = scenarioStore{bestID};
bestEffectors = configStore{bestID};
for k = 1:num_tests
    replayStartPos = scenariosToReplay{k};
    uasArray = UAS.empty(0, numAdversaries);
    for u = 1:numAdversaries
        uasArray(u) = UAS(15, replayStartPos(u,:), assets(1).location, 'Linear', 25);
    end
    sim = simulator(mapObj, AOR, uasArray, bestEffectors, sensors, assets, ...
        'tps', 20, 'animate', true, 'fadePings', true, 'nfzs', polyshape.empty, ...
        'resetGraphics', true, 'animationMultiplier', 5, 'costConfig', costConfig);
    sim.runSim();
    title(sprintf('Run %d/%d | Cost: $%.2f', k, num_tests, costDetailsStore{bestID}(k)));
end