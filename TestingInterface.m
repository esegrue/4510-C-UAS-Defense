clear; clc; close all;

% MONTE CARLO SETTINGS
maxConfigs = 100;
num_tests = 10;
delta = 100; 

configStore = cell(maxConfigs, 1);      
scenarioStore = cell(maxConfigs, 1);    
costDetailsStore = cell(maxConfigs, 1); 

CostperCombo = nan(maxConfigs, 1);
ReliabilityScore = nan(maxConfigs, 1); 
LCB = nan(maxConfigs, 1); UCB = nan(maxConfigs, 1); 
separation = nan(maxConfigs, 1);

% SIMULATION PARAMETERS
% defining map
mapL = 100; mapW = 100; mapBounds = [0 mapL 0 mapW];
mapObj = map(mapL, mapW, 1); 
mapObj.generateTerrain('Hills', 30); 
xlims = [0 mapL]; ylims = [0 mapW];

% initializing kill locations
killStore = cell(maxConfigs, 1);

% simulation settings
numAssets = 1; numSensors = 2; numEffectors = 3; numAdversaries = 3;

% loading effector positions and adversary paths banks
load("effector_posn_bank.mat")
% load("adversary_paths_bank.mat")
load("adversary_paths_bank.mat") %NOT IMPLEMENTED YET

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
effectorRange = 20; 
effectorStructTemplate = struct('location', [0,0], 'range', effectorRange);

% define AOR - what do we want to do with this?
AORo = (assets(1).location + assets(2).location)/2; AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], ...
                [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);

% defining cost function
costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250);
reliabilityThreshold = 90; 

% CONFIG LOOP
numConfigs = 0;
while numConfigs < maxConfigs
    numConfigs = numConfigs + 1;
    
    % generating effectors
    effPos = effector_posns_bank(randi(length(effector_posns_bank),[1 numEffectors]),:);
    currentEffectors = repmat(effectorStructTemplate, numEffectors, 1);
    for e = 1:numEffectors
        currentEffectors(e).location = effPos(e, :);
    end
    configStore{numConfigs} = currentEffectors;

    % initialize scenario storage
    runCosts = zeros(num_tests, 1); 
    runStarts = cell(num_tests, 1);
    runFailures = 0;

    % storage for kills in this config
    currentConfigKills = [];
    
    % SCENARIO LOOP
    for j = 1:num_tests
        % generating adversary
        starts = ingressPosns(xlims, ylims, numAdversaries);
        
        % generating seed
        runSeed = numConfigs*1000 + j;
        rng(runSeed);

        uasArray = UAS.empty(0, numAdversaries);
        for k = 1:numAdversaries
            uasArray(k) = UAS(15, starts(k,:), assets(1).location, 'HybridAStar', 25);
        end
        
        % simulating
        sim = simulator(mapObj, AOR, uasArray, currentEffectors, sensors, assets, 'tps', 20, 'animate', false, 'nfzs', polyshape.empty, 'resetGraphics', true, 'costConfig', costConfig);
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
    
    % storing config results
    scenarioStore{numConfigs} = runStarts;
    costDetailsStore{numConfigs} = runCosts;
    CostperCombo(numConfigs) = mean(runCosts);
    killStore{numConfigs} = currentConfigKills;

    % CONDITIONAL CONVERGENCE
    % calculating config reliability
    currentReliability = 100 * (1 - (runFailures / num_tests));
    ReliabilityScore(numConfigs) = currentReliability;
    
    % Student's t-distribution
    SD = std(runCosts, 1); 
    SE = SD/sqrt(num_tests); 
    alpha = 0.1;
    tcrit = tinv(1 - alpha/2, num_tests - 1);
    LCB(numConfigs) = CostperCombo(numConfigs) - tcrit*SE;
    UCB(numConfigs) = CostperCombo(numConfigs) + tcrit*SE;
    
    % filtering unreliable configs
    validMask = find(ReliabilityScore(1:numConfigs) >= reliabilityThreshold);
    
    % comparing configs
    if isempty(validMask) || length(validMask) == 1
        separation(numConfigs) = -inf;
        fprintf('Iter %d: No comparison (Reliability: %.1f%%)\n', numConfigs, currentReliability);
    else
        validCosts = CostperCombo(validMask);
        [~, bestIdxInValid] = min(validCosts);
        bestGlobalID = validMask(bestIdxInValid);
        
        rivalsMask = validMask(validMask ~= bestGlobalID);
        separation(numConfigs) = min(LCB(rivalsMask)) - UCB(bestGlobalID);
        fprintf('Iter %d: Valid Best #%d ($%.0f) vs Rival ($%.0f) | Sep: %.2f\n', numConfigs, bestGlobalID, CostperCombo(bestGlobalID), min(CostperCombo(rivalsMask)), separation(numConfigs));
    end
    
    if separation(numConfigs) > delta
        fprintf('Monte Carlo Simulation converged!\n');
        break
    end
end

% CONFIG SELECTION
validCandidates = find(ReliabilityScore(1:numConfigs) >= reliabilityThreshold);
if isempty(validCandidates)
    fprintf('\nWARNING: Target Reliability NOT Met. Selecting best available.\n');
    [~, bestID] = max(ReliabilityScore(1:numConfigs));
else
    [minCost, idx] = min(CostperCombo(validCandidates));
    bestID = validCandidates(idx);
    fprintf('\nBest Configuration: Iteration %d\n', bestID);
    fprintf('Reliability: %.1f%% | Avg Cost: $%.2f\n', ReliabilityScore(bestID), minCost);
end

% VISUALIZING
% kill zone heatmap
figure('Name', 'Kill Zone Heatmap', 'Position', [100 100 600 400]);
hold on; axis equal; box on;
colormap('jet'); % Red = Hot (High Kills), Blue = Cold

% aggregating data (top 10 configs)
[sortedCosts, sortIdx] = sort(CostperCombo(1:numConfigs)); % Sort by cost ascending
topN = min(10, numConfigs);
topIndices = sortIdx(1:topN);
aggregateKills = [];
for i = 1:topN
    cfgID = topIndices(i);
    if ~isempty(killStore{cfgID})
        aggregateKills = [aggregateKills; killStore{cfgID}];
    end
end

% drawing terrain contours
if ~isempty(mapObj.terrain)
    % Draw gray contour lines for elevation
    [C_terr, h_terr] = contour(mapObj.terrain.X, mapObj.terrain.Y, mapObj.terrain.Z, 10);
    h_terr.LineColor = [0.6 0.6 0.6]; % Gray lines
    h_terr.LineWidth = 1;
    clabel(C_terr, h_terr, 'Color', [0.4 0.4 0.4], 'FontSize', 8);
end

% saving data
if ~isempty(aggregateKills)
    numBins = 30; % Resolution
    x_edges = linspace(0, mapL, numBins+1);
    y_edges = linspace(0, mapW, numBins+1);
    
    % calculate 2D counts
    [N, Xedges, Yedges] = histcounts2(aggregateKills(:,1), aggregateKills(:,2), x_edges, y_edges);
    
    % calculate centers
    Xcenters = (Xedges(1:end-1) + Xedges(2:end))/2;
    Ycenters = (Yedges(1:end-1) + Yedges(2:end))/2;
    
    maxKills = max(N(:));
    
    levels = linspace(0, maxKills, maxKills+1); % Discrete integer steps
    
    % Plot filled contours
    [C_kill, h_kill] = contourf(Xcenters, Ycenters, N', levels, 'LineStyle', 'none');
    
    % Transparency
    h_kill.FaceAlpha = 0.6; 
    
    % Force the color map to use the full range (Blue to Red)
    colormap('jet'); 
    
    if maxKills > 0
        clim([0 maxKills]); % Forces max value to be the "Reddest" color
        % Note: If using MATLAB older than 2022a, use: caxis([0 maxKills]);
    end
    
    cb = colorbar;
    cb.Label.String = 'Kill Density (Interceptions)';
    % Fix colorbar ticks to show integers if counts are low
    if maxKills <= 5
        cb.Ticks = 0:maxKills;
    end
else
    fprintf('No kills recorded in top configurations.\n');
end

% plotting assets and AOR
% Draw AOR boundary
plot(AOR, 'FaceColor', 'none', 'EdgeColor', 'w', 'LineStyle', '--', 'LineWidth', 2);
plot(AOR, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineStyle', '--', 'LineWidth', 1); % Black outline for contrast

for a = 1:length(assets)
    plot(assets(a).location(1), assets(a).location(2), 'sq', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'g', 'Color', 'k', 'LineWidth', 2);
end

% Formatting
axis(mapBounds);
title({sprintf('Kill Zone Heatmap (Top %d Configs)', topN)});
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
set(gca, 'Layer', 'top'); % Ensures grid lines (if on) and ticks are on top
hold off;


% risk vs. reward plot
figure('Name', 'Trade-off Analysis', 'Position', [100 100 600 400]);
hold on; grid on;

s1 = scatter(CostperCombo(1:numConfigs), ReliabilityScore(1:numConfigs), 70, 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Configurations');
s2 = scatter(CostperCombo(bestID), ReliabilityScore(bestID), 150, 'r', 'filled', 'p', 'DisplayName', 'Best Config');
xlabel('Average Cost ($)');
ylabel('Reliability (%)');
title('Cost vs. Reliability');
hold off
legend([s1, s2],'Location', 'best');

xline(mean(CostperCombo(1:numConfigs)), '--k', 'Alpha', 0.3, 'DisplayName', 'Mean Cost');
yline(reliabilityThreshold, 'g--', 'LineWidth', 2, 'DisplayName', 'Reliability Threshold');


%  top configs position plot
figure('Name', 'Position Analysis', 'Position', [100 100 600 400]);
hold on; axis equal; box on;

% Access terrain data directly from the map object
if ~isempty(mapObj.terrain)
    % Create filled contours for elevation (Topographic view)
    [C, h] = contourf(mapObj.terrain.X, mapObj.terrain.Y, mapObj.terrain.Z, 15);
    clabel(C, h, 'Color', 'w', 'FontSize', 8); % Label elevation lines
    colormap(summer); % Green/Yellow terrain map
    c = colorbar;
    c.Label.String = 'Elevation (m)';
else
    xlim([0 mapL]); ylim([0 mapW]);
end

for a = 1:length(assets)
    plot(assets(a).location(1), assets(a).location(2), 'sq', ...
        'MarkerSize', 15, 'MarkerFaceColor', 'g', 'Color', 'k', 'LineWidth', 2, ...
        'DisplayName', 'Asset');
end

colorMap = flipud(jet(topN + 2)); 
for i = 1:topN
    cfgID = topIndices(i);
    effs = configStore{cfgID};
    effLocs = vertcat(effs.location);
    scatter(effLocs(:,1), effLocs(:,2), 120, colorMap(i,:), 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1, ...
        'DisplayName', sprintf('Rank #%d (Avg Cost $%.0f)', i, sortedCosts(i)));
end

% Formatting
title(sprintf('Top 10 Configurations'));
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
legend('Location', 'eastoutside');
grid off;
hold off;


% best scenario cost histogram
figure('Name', 'Reliability Analysis', 'Position', [100 100 600 400]);
hold on; grid on;
histogram(costDetailsStore{bestID},'FaceColor', 'g');
xline(costConfig.asset, 'r--', 'LineWidth', 2, 'Label', 'Asset Loss');
xline(costConfig.leak, 'k--', 'LineWidth', 1.5, 'Label', 'Leak');
xline(costConfig.effector * numAdversaries, 'b--', 'LineWidth', 1.5, 'Label', 'Ideal');
title(sprintf('Best Config (#%d) Cost Distribution', bestID));
xlabel('Scenario Cost ($)'); ylabel('Frequency');

% winning config simulation
figure('Name', 'Winning Configuration', 'Position', [100 100 600 400]);
hold on; grid on; axis equal; box on;
fprintf('\nReplaying Winning Configuration...\n');

scenariosToReplay = scenarioStore{bestID};
bestEffectors = configStore{bestID};
for k = 1:num_tests
    % obtaining replay seed
    replaySeed = bestID*1000 + k;
    rng(replaySeed);

    % loading settings
    replayStartPos = scenariosToReplay{k};
    uasArray = UAS.empty(0, numAdversaries);
    for u = 1:numAdversaries
        uasArray(u) = UAS(15, replayStartPos(u,:), assets(1).location, 'HybridAStar', 25);
    end
    sim = simulator(mapObj, AOR, uasArray, bestEffectors, sensors, assets, 'tps', 20, 'animate', true, 'fadePings', true, 'nfzs', polyshape.empty, 'resetGraphics', true, 'animationMultiplier', 5, 'costConfig', costConfig);
    sim.runSim();
    title(sprintf('Run %d/%d | Cost: $%.2f', k, num_tests, costDetailsStore{bestID}(k)));
end