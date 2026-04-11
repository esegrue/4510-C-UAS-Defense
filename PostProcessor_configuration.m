clear; clc; close all;

%% USER CONFIGURATION
% -------------------

% VISUALIZATION SETTINGS
visConfig.topN = 5; % number of top configs to analyze
visConfig.heatmapBins = 30; % resolution of heatmap
visConfig.contourLevels = 10; % number of terrain contour lines

% REPLAY SETTINGS
replayConfig.mode = 'Best'; % replay mode ('Specific')
replayConfig.specificID = 1; % replay ID (only use if mode is 'Specific')
replayConfig.tps = 20; % replay simulation speed (ticks/sec)
replayConfig.animMult = 5; % animation speed multiplier
replayConfig.maxReplays = 5;

%% LOAD & STITCH BATCH DATA
% -------------------------

disp('Please select the _Environment.mat file AND all _Batch files for your scenario...');
[fileNames, pathName] = uigetfile('*.mat', 'Select Environment and Batch files', 'MultiSelect', 'on');

if isequal(fileNames,0)
   error('No files selected. Exiting.');
end

if ischar(fileNames)
    fileNames = {fileNames}; 
end

fprintf('Analyzing %d selected files...\n', length(fileNames));

% Find and load the environment file
envFound = false;
for i = 1:length(fileNames)
    if contains(fileNames{i}, 'Environment')
        fprintf(' -> Loading static Environment data from %s...\n', fileNames{i});
        fullPath = fullfile(pathName, fileNames{i});
        envData = load(fullPath);
        
        mapObj = envData.MapData;
        asset = envData.Asset;
        sensors = envData.Sensors;
        metadata = envData.Metadata;
        costConfig = metadata.CostConfig;
        
        envFound = true;
        break; 
    end
end

if ~envFound
    error('CRITICAL: Environment file not found. Please ensure you select the [Prefix]_Environment.mat file along with your batches.');
end

% Load and stitch batch files
combinedConfigs = [];
for i = 1:length(fileNames)
    % Skip the environment file and the summary file if selected
    if contains(fileNames{i}, 'Environment') || contains(fileNames{i}, 'Summary')
        continue; 
    end
    
    fullPath = fullfile(pathName, fileNames{i});
    data = load(fullPath); 
    
    if isfield(data, 'Configs')
        batchConfigs = data.Configs(:)';
        
        validIdx = arrayfun(@(x) ~isempty(x.ID) && x.ID > 0, batchConfigs);
        batchConfigs = batchConfigs(validIdx);
        
        combinedConfigs = [combinedConfigs, batchConfigs];
    else
        warning('Unrecognized data format in file: %s. Skipping.', fileNames{i});
    end
end

% Sort stitched configurations by their ID
[~, sortIdx] = sort([combinedConfigs.ID]);
combinedConfigs = combinedConfigs(sortIdx);

% Reconstruct the standard SimResults structure in memory 
SimResults.Configs = combinedConfigs;
SimResults.MapData = mapObj;
SimResults.Asset = asset;
SimResults.Sensors = sensors;
SimResults.Metadata = metadata;
numConfigs = length(combinedConfigs);
SimResults.NumConfigsRun = numConfigs;

load('adversary_paths_bank.mat', 'adversary_paths_bank');

% arrays for plotting
CostperCombo = [SimResults.Configs.CostMean];
ReliabilityScore = [SimResults.Configs.Reliability];

% Extract separation history for convergence analysis
if isfield(SimResults.Configs, 'Separation')
    SeparationHistory = [SimResults.Configs.Separation];
else
    SeparationHistory = nan(1, numConfigs);
end

% find the overall Best ID across all batches
validMask = ReliabilityScore >= metadata.ReliabilityThreshold;
if ~any(validMask)
    [~, bestID] = max(ReliabilityScore);
else
    validCosts = CostperCombo(validMask);
    [~, minIdx] = min(validCosts);
    validIDs = find(validMask);
    bestID = validIDs(minIdx);
end
SimResults.BestConfigID = bestID;

fprintf('Successfully stitched %d configurations. Best Config is #%d.\n', numConfigs, bestID);

% set bounds
topN = min(visConfig.topN, numConfigs);
mapBounds = SimResults.Metadata.MapBounds;
mapL = mapBounds(2); mapW = mapBounds(4);

%% KILL ZONE HEATMAP
% ------------------

figure('Name', 'Kill Zone Heatmap', 'Position', [100 100 600 400]);
hold on; axis equal; box on;
colormap('jet'); 

% aggregate data (top N configs)
[sortedCosts, sortIdx] = sort(CostperCombo); 
topIndices = sortIdx(1:topN);

aggregateKills = [];
for i = 1:topN
    cfgID = topIndices(i);
    kLocs = SimResults.Configs(cfgID).KillLocations;
    if ~isempty(kLocs)
        aggregateKills = [aggregateKills; kLocs];
    end
end

% terrain contours
if ~isempty(mapObj.terrain)
    [C_terr, h_terr] = contour(mapObj.terrain.X, mapObj.terrain.Y, mapObj.terrain.Z, visConfig.contourLevels);
    h_terr.LineColor = [0.6 0.6 0.6];
    h_terr.LineWidth = 1;
end

% plot heatmap
if ~isempty(aggregateKills)
    x_edges = linspace(0, mapL, visConfig.heatmapBins+1);
    y_edges = linspace(0, mapW, visConfig.heatmapBins+1);
    
    [N, Xedges, Yedges] = histcounts2(aggregateKills(:,1), aggregateKills(:,2), x_edges, y_edges);
    Xcenters = (Xedges(1:end-1) + Xedges(2:end))/2;
    Ycenters = (Yedges(1:end-1) + Yedges(2:end))/2;
    
    maxKills = max(N(:));
    levels = linspace(0, maxKills, maxKills+1); 
    
    [C_kill, h_kill] = contourf(Xcenters, Ycenters, N', levels, 'LineStyle', 'none');
    h_kill.FaceAlpha = 0.6; 
    
    if maxKills > 0; clim([0 maxKills]); end
    
    cb = colorbar;
    cb.Label.String = 'Kill Density (Interceptions)';
else
    fprintf('No kills recorded in top configurations.\n');
end

% plot asset
plot(asset.location(1), asset.location(2), 'sq', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'g', 'Color', 'k', 'LineWidth', 2);

axis(mapBounds);
title({sprintf('Kill Zone Heatmap (Top %d Configs)', topN)});
xlabel('X Coordinate (m)'); ylabel('Y Coordinate (m)');
set(gca, 'Layer', 'top'); 
hold off;


%% RISK VS REWARD
% ---------------

figure('Name', 'Trade-off Analysis', 'Position', [100 100 600 400]);
hold on; grid on;

s1 = scatter(CostperCombo, ReliabilityScore, 70, 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Configurations');
s2 = scatter(CostperCombo(bestID), ReliabilityScore(bestID), 150, 'r', 'filled', 'p', 'DisplayName', 'Best Config');
xlabel('Average Cost ($)');
ylabel('Reliability (%)');
title('Cost vs. Reliability');
hold off
legend([s1, s2],'Location', 'best');

xline(mean(CostperCombo), '--k', 'Alpha', 0.3, 'DisplayName', 'Mean Cost');
yline(SimResults.Metadata.ReliabilityThreshold, 'g--', 'LineWidth', 2, 'DisplayName', 'Reliability Threshold');


%% POSITION PATTERN ANALYSIS
% --------------------------

figure('Name', 'Position Pattern Analysis', 'Position', [100 100 600 400]);
hold on; axis equal; box on;

% plot terrain background
if ~isempty(mapObj.terrain)
    [C, h] = contourf(mapObj.terrain.X, mapObj.terrain.Y, mapObj.terrain.Z, visConfig.contourLevels + 5);
    clabel(C, h, 'Color', 'w', 'FontSize', 8); 
    colormap(summer); 
    c = colorbar; c.Label.String = 'Elevation (m)';
else
    xlim([0 mapL]); ylim([0 mapW]);
end

% plot asset
plot(asset.location(1), asset.location(2), 'sq', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Asset');

% plot effector positions
colorMap = flipud(jet(topN + 2)); 
for i = 1:topN
    cfgID = topIndices(i);
    effs = SimResults.Configs(cfgID).Effectors;
    effLocs = vertcat(effs.location);
    scatter(effLocs(:,1), effLocs(:,2), 120, colorMap(i,:), 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1, ...
        'DisplayName', sprintf('Rank #%d (Avg Cost $%.0f)', i, sortedCosts(i)));
end

title(sprintf('Top %d Configurations', topN));
xlabel('X Coordinate (m)'); ylabel('Y Coordinate (m)');
legend('Location', 'eastoutside');
grid off; hold off;


%% SCENARIO RELIABILITY DISTRIBUTION
% ----------------------------------

figure('Name', 'Reliability Distribution', 'Position', [100 100 600 400]);
hold on; grid on;
bestTrials = SimResults.Configs(bestID).Trials;
bestCosts = [bestTrials.Cost];

histogram(bestCosts,'FaceColor', 'g');
xline(costConfig.asset, 'r--', 'LineWidth', 2, 'Label', 'Asset Loss');
xline(costConfig.leak, 'k--', 'LineWidth', 1.5, 'Label', 'Leak');
xline(costConfig.effector * SimResults.Metadata.NumAdversaries, 'b--', 'LineWidth', 1.5, 'Label', 'Ideal');
title(sprintf('Best Config (#%d) Cost Distribution', bestID));
xlabel('Scenario Cost ($)'); ylabel('Frequency');


%% MONTE CARLO TRIAL CONVERGENCE
% ------------------------------

figure('Name', 'Simulation Convergence Analysis', 'Position', [200 200 700 450]);
hold on; grid on; box on;

% trial data for the best configuration
bestTrials = SimResults.Configs(bestID).Trials;
N_tests = length(bestTrials);
trialCosts = [bestTrials.Cost];
assetCost = costConfig.asset;

% Preallocate tracking arrays
cumMean = zeros(1, N_tests);
cumLCB = zeros(1, N_tests);
cumUCB = zeros(1, N_tests);
cumReliability = zeros(1, N_tests);

% Calculate stats trial-by-trial
alpha = SimResults.Metadata.MCSettings.confidenceAlpha;
failures = 0;

for n = 1:N_tests
    currentCosts = trialCosts(1:n);
    cumMean(n) = mean(currentCosts);
    
    if currentCosts(end) >= assetCost
        failures = failures + 1;
    end
    cumReliability(n) = 100 * (1 - (failures / n));
    
    if n > 1
        currStd = std(currentCosts);
        currSE = currStd / sqrt(n);
        tcrit = tinv(1 - alpha/2, n - 1);
        cumLCB(n) = cumMean(n) - tcrit * currSE;
        cumUCB(n) = cumMean(n) + tcrit * currSE;
    else
        cumLCB(n) = cumMean(n);
        cumUCB(n) = cumMean(n);
    end
end

% plotting cost
yyaxis left
x_fill = [1:N_tests, fliplr(1:N_tests)];
y_fill = [cumLCB, fliplr(cumUCB)];
fill(x_fill, y_fill, 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', '90% Confidence Interval');

plot(1:N_tests, cumMean, 'b-', 'LineWidth', 2, 'DisplayName', 'Rolling Mean Cost');
ylabel('Cost ($)');
ylim([min(cumLCB(2:end))*0.9, max(cumUCB(2:end))*1.1]); % Ignore n=1 for scaling

% plotting reliability
yyaxis right
plot(1:N_tests, cumReliability, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Rolling Reliability');
yline(SimResults.Metadata.ReliabilityThreshold, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Threshold');
ylabel('Reliability (%)');
ylim([0 100]);

title(sprintf('Simulation Convergence (Config #%d)', bestID));
xlabel('Number of Monte Carlo Trials');
xlim([1 N_tests]);
legend('Location', 'best');
hold off;


%% CONVERGENCE SEPARATION ANALYSIS
% --------------------------------

figure('Name', 'Convergence Analysis', 'Position', [150 150 600 400]);
hold on; grid on; box on;

if ~all(isnan(SeparationHistory))
    validSepIdx = find(SeparationHistory ~= -inf & ~isnan(SeparationHistory));
    
    if ~isempty(validSepIdx)
        plot(validSepIdx, SeparationHistory(validSepIdx), '-o', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Separation');
        
        if isfield(metadata, 'MCSettings') && isfield(metadata.MCSettings, 'minSeparation')
            deltaThreshold = metadata.MCSettings.minSeparation;
            yline(deltaThreshold, 'r--', 'LineWidth', 2, 'DisplayName', 'Convergence Threshold');
        end
        
        legend('Location', 'best');
    else
        text(0.5, 0.5, 'No valid comparisons found yet', 'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
else
    text(0.5, 0.5, 'Separation data not found in files', 'Units', 'normalized', 'HorizontalAlignment', 'center');
end

xlabel('Configuration Iteration');
ylabel('Separation ($)');
title('Configuration Convergence History');
hold off;


%% REPLAY CONFIGURATION
% ---------------------

figure('Name', 'Replay', 'Position', [100 100 600 400]);
hold on; grid on; axis equal; box on;

% determine which config to replay
if strcmpi(replayConfig.mode, 'Specific')
    targetID = replayConfig.specificID;
    if targetID > numConfigs || targetID < 1
        warning('Specific ID %d out of range. Replaying Best ID instead.', targetID);
        targetID = bestID;
    end
else
    targetID = bestID;
end

fprintf('\nReplaying Configuration #%d...\n', targetID);

% prepare replay data
replayEffectors = SimResults.Configs(targetID).Effectors;
trialsToReplay = SimResults.Configs(targetID).Trials;
numReplays = min(length(trialsToReplay), replayConfig.maxReplays);

% extract weapon config
wC = SimResults.Metadata.weaponConfig;
wDwell = wC.directEnergyDwellTime;
wKinSpd = wC.kineticProjectileSpeed;
wKinShots = wC.kineticShotsPerVolley;
wKinHitP = wC.kineticHitProbability;
wKinRefire = wC.kineticRefireTime;
wKinTol = wC.projectileHitTolerance;
wFermi = wC.kineticUseFermiModel;

for k = 1:numReplays

    % obtaining replay seed and inputs
    simSeed = trialsToReplay(k).Seed;
    localStream = RandStream('mt19937ar', 'Seed', simSeed);

    numAdversaries = SimResults.Metadata.NumAdversaries;
    
    % redefine UAS
    uasArray = UAS.empty(0, numAdversaries);
    
    uSpeed = SimResults.Metadata.advConfig.speed;
    uTurn = SimResults.Metadata.advConfig.turnRadius;
    uPlan = SimResults.Metadata.advConfig.planner;
    uAlt_default = SimResults.Metadata.advConfig.altitude;
    uSearchRange = SimResults.Metadata.advConfig.searchRange;
    
    for u = 1:numAdversaries
        pIdx = randi(localStream, length(adversary_paths_bank));
        path = adversary_paths_bank{1, 1, pIdx};
        
        startX = path(1, 1);
        startY = path(1, 2);
        startZ = uAlt_default;
        
        groundElevation = mapObj.getElevation(startX, startY);
        while groundElevation >= startZ
            startZ = startZ + 1;
        end
        startPos = [startX, startY, startZ];
        
        uasArray(u) = UAS(uSpeed, startPos, asset.location, uPlan, startZ, uTurn, "adversary_path", path, "searchRange", uSearchRange);
    end
    
    % redefine simulator
    sim = simulator(mapObj, uasArray, replayEffectors, sensors, asset, 'tps', replayConfig.tps, 'animate', true, 'fadePings', true, 'resetGraphics', true, 'animationMultiplier', replayConfig.animMult, 'costConfig', costConfig, ...
        'randomStream', localStream, ...
        'directEnergyDwellTime', wDwell, ...
        'kineticProjectileSpeed', wKinSpd, ...
        'kineticShotsPerVolley', wKinShots, ...
        'kineticHitProbability', wKinHitP, ...
        'kineticRefireTime', wKinRefire, ...
        'projectileHitTolerance', wKinTol, ...
        'kineticUseFermiModel', wFermi);
        
    sim.runSim();
    title(sprintf('Replay Config #%d | Run %d/%d | Cost: $%.2f', ...
        targetID, k, numReplays, trialsToReplay(k).Cost));
end