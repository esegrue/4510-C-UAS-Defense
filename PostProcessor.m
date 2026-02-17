clear; clc; close all;

%% USER CONFIGURATION
% -------------------

% LOADING DATA
visConfig.fileToLoad = ''; % or provide a filename ('SimData_XXXXXXXX_XXXXXX.mat')

% VISUALIZATION SETTINGS
visConfig.topN = 10; % number of top configs to analyze
visConfig.heatmapBins = 30; % resolution of heatmap
visConfig.contourLevels = 10; % number of terrain contour lines

% REPLAY SETTINGS
replayConfig.mode = 'Best'; % replay mode ('Specific')
replayConfig.specificID = 1; % replay ID (only use if mode is 'Specific')
replayConfig.tps = 20; % replay simulation speed (ticks/sec)
replayConfig.animMult = 5; % animation speed multiplier

%% LOAD DATA
% ----------

if isempty(visConfig.fileToLoad)
    files = dir('SimData_*.mat');
    if isempty(files)
        error('No simulation data files found.');
    end
    [~, idx] = max([files.datenum]); 
    fileToLoad = files(idx).name;
else
    fileToLoad = visConfig.fileToLoad;
    if ~exist(fileToLoad, 'file')
        error('Specified file not found: %s', fileToLoad);
    end
end

fprintf('Loading data from: %s\n', fileToLoad);
load(fileToLoad, 'SimResults');

% unpacking variables
mapObj = SimResults.MapData;
asset = SimResults.Asset;
sensors = SimResults.Sensors;
costConfig = SimResults.Metadata.CostConfig;
numConfigs = SimResults.NumConfigsRun;
bestID = SimResults.BestConfigID;

% set bounds
topN = min(visConfig.topN, numConfigs);
mapBounds = SimResults.Metadata.MapBounds;
mapL = mapBounds(2); mapW = mapBounds(4);

% arrays for plotting
CostperCombo = [SimResults.Configs.CostMean];
ReliabilityScore = [SimResults.Configs.Reliability];


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
% ---------------------------

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
numReplays = length(trialsToReplay);

for k = 1:numReplays

    % obtaining replay seed and inputs
    replaySeed = trialsToReplay(k).Seed;
    rng(replaySeed, 'twister');

    replayStartPos = trialsToReplay(k).Starts;
    numAdversaries = SimResults.Metadata.NumAdversaries;
    
    % redefine UAS
    uasArray = UAS.empty(0, numAdversaries);
    
    if isfield(SimResults.Metadata, 'advConfig')
        uSpeed = SimResults.Metadata.advConfig.speed;
        uTurn = SimResults.Metadata.advConfig.turnRadius;
        uPlan = SimResults.Metadata.advConfig.planner;
        uAlt = SimResults.Metadata.advConfig.altitude;
    else
        uSpeed = 15; uTurn = 25; uPlan = 'HybridAStar';
    end
    
    for u = 1:numAdversaries
        uasArray(u) = UAS(uSpeed, replayStartPos(u,:), asset.location, uPlan, uAlt, uTurn);
    end
    
    % redefine simulator
    sim = simulator(mapObj, uasArray, replayEffectors, sensors, asset, 'tps', replayConfig.tps, 'animate', true, 'fadePings', true, 'nfzs', polyshape.empty, 'resetGraphics', true, 'animationMultiplier', replayConfig.animMult, 'costConfig', costConfig);
        
    sim.runSim();
    title(sprintf('Replay Config #%d | Run %d/%d | Cost: $%.2f', ...
        targetID, k, numReplays, trialsToReplay(k).Cost));
end