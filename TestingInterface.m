clear; clc; close all;

% simulation parameters
mapL = 100;
mapW = 100;
mapObj = map(mapL, mapW);
xlims = [0 mapL]; ylims = [0 mapW];

% counts for all scenarios
numAssets = 2;
numSensors = 2;
numEffectors = 2;
numAdversaries = 3;

% defining asset
assetStructTemplate = struct('location', [0,0]);
assets = repmat(assetStructTemplate, numAssets, 1);
assets(1).location = [30, 60];
assets(2).location = [60, 45];

% defining sensors
sensorRange = 10;
params = struct('d50', 2*sensorRange, 'k', 2, 'pings', 6, 'duration', 0.5);
peakGain = 1; 
boresight = 0; 
beamwidth = 360;

% initializing sensors struct
sensorStructTemplate = struct('location', [0,0], 'range', sensorRange, ...
    'params', params, 'xg', [], 'yg', [], 'P', []);
sensors = repmat(sensorStructTemplate, numSensors, 1);

% generating sensor contours
sensorLocs = [assets(1).location; assets(2).location];
for i = 1:numSensors
    sensors(i).location = sensorLocs(i,:);
    [xg, yg] = meshgrid(0:1:mapL, 0:1:mapW);
    dx = xg - sensors(i).location(1);
    dy = yg - sensors(i).location(2);
    d = sqrt(dx.^2 + dy.^2);
    d50 = params.d50; k_val = params.k;
    Pd = 1 ./ (1 + exp((d - d50)/k_val));
    sensors(i).xg = xg; sensors(i).yg = yg; sensors(i).P = peakGain .* Pd; 
end

% defining map
NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);
AORo = (assets(1).location + assets(2).location)/2;
AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], ...
                [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);

% Monte Carlo settings
max_configs = 100;
num_tests = 10;

% initializing simulation settings/results
configStore = cell(max_configs, 1);      
scenarioStore = cell(max_configs, 1);    
costDetailsStore = cell(max_configs, 1); 

CostperCombo = nan(max_configs, 1);
LCB = nan(max_configs, 1); % lower confidence bound
UCB = nan(max_configs, 1); % upper confidence bound
separation = nan(max_configs, 1);

num_configs = 0;
delta = 10; % larger delta = tighter convergence

% initializing effectors struct
effectorRange = 10;
effectorStructTemplate = struct('location', [0,0], 'range', effectorRange);

% Monte Carlo loop
while num_configs < max_configs
    num_configs = num_configs + 1;
    
    effPos = effectorPosnsGenerator(xlims, ylims, numEffectors); % sampling effector locations
    
    % assigning effector locations
    currentEffectors = repmat(effectorStructTemplate, numEffectors, 1);
    for e = 1:numEffectors
        currentEffectors(e).location = effPos(e, :);
    end
    
    configStore{num_configs} = currentEffectors; % storing simulation configurations

    runCosts = zeros(num_tests, 1); % initializing scenario results

    runStarts = cell(num_tests, 1); % initializing scenario storage
    
    % scenario loop
    parfor j = 1:num_tests
        starts = ingressPosns(xlims, ylims, numAdversaries); % adversary ingress locations
        
        % defining UAS class
        uasArray = UAS.empty(0, numAdversaries);
        for k = 1:numAdversaries
            uasArray(k) = UAS(15, starts(k,:), assets(1).location, 'Linear');
        end
        
        % simulating a scenario
        sim = simulator(mapObj, AOR, uasArray, currentEffectors, sensors, assets, 'tps', 20, 'animate', false, 'nfzs', NFZ1, 'resetGraphics', true);
        runResults = sim.runSim();
        
        runCosts(j) = runResults.cost; % scenario results
        runStarts{j} = starts; % storing scenario settings
    end
    
    % storing configuration settings/results
    scenarioStore{num_configs} = runStarts;
    costDetailsStore{num_configs} = runCosts;
    
    CostperCombo(num_configs) = sum(runCosts, 'all')/num_tests; % configuration estimated cost
    SD = std(runCosts, 1); % configuration results standard deviation
    SE = SD/sqrt(num_tests); % standard error of the mean
    
    % students t-distribution
    alpha = 0.1; % 90% confidence
    tcrit = tinv(1 - alpha/2, num_tests - 1);
    LCB(num_configs) = CostperCombo(num_configs) - tcrit*SE;
    UCB(num_configs) = CostperCombo(num_configs) + tcrit*SE;

    % comparing configurations
    runsMask = find(~isnan(CostperCombo));
    [~, b] = min(CostperCombo(runsMask));
    rivals = setdiff(runsMask, b);
    if isempty(rivals)
        separation(num_configs) = -inf;
    else
        separation(num_configs) = min(LCB(rivals)) - UCB(b); 
    end
    
    % convergence
    if separation(num_configs) > delta
        break
    end
end

% best configuration
[minCost, minCostID] = min(CostperCombo(1:num_configs));
fprintf('Best Configuration: Iteration %d (Avg Cost: %.2f)\n', minCostID, minCost);

%% Animation

% reloading configuration and scenario settings
bestEffectors = configStore{minCostID};
scenariosToReplay = scenarioStore{minCostID};
avgCost = CostperCombo(minCostID);

% plotting
figure(1);
title(sprintf('Best Config (Iter %d) | Scenarios: %d | Avg Cost: $%.2f', ...
    minCostID, num_tests, avgCost));

fprintf('\nReplaying all %d scenarios for Configuration %d...\n', num_tests, minCostID);
fprintf('Average Cost for this batch: $%.2f\n', avgCost);

for k = 1:num_tests
    
    replayStartPos = scenariosToReplay{k}; % reload scenarios
    
    % redefining UAS
    uasArray = UAS.empty(0, numAdversaries);
    for u = 1:numAdversaries
        uasArray(u) = UAS(15, replayStartPos(u,:), assets(1).location, 'Linear');
    end
    
    % simulating a scenario
    if k == 1
        shouldReset = true;
    else
        shouldReset = false;
    end

    sim = simulator(mapObj, AOR, uasArray, bestEffectors, sensors, assets, 'tps', 20, 'animate', true, 'fadePings', true, 'nfzs', NFZ1, 'resetGraphics', true, 'animationMultiplier', 5, 'hideClock', false);
    sim.runSim();
    
    title(sprintf('Config %d | Run %d/%d | Avg Cost: $%.2f', ...
        minCostID, k, num_tests, avgCost));
end

fprintf('Batch Replay Complete.\n');