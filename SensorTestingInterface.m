clear; clc; close all;

%% CONFIGURATION

% Map
mapConfig.L = 100;                  % map length (units)
mapConfig.W = 100;                  % map width (units)
mapConfig.terrainType = 'Hills';    % 'Flat' or 'Hills'
mapConfig.terrainMag = 5;           % peak terrain height (Hills only)

% Asset
assetConfig.location = [30, 60];    % [X, Y]

% Sensors
sensConfig.count = 3;               % number of sensors per configuration
sensConfig.range = 15;              % detection radius (units)
sensConfig.minSpacing = 10;         % minimum distance between sensors (units)
sensConfig.params = struct( ...
    'd50', 25, ...                  % distance at 50% detection probability
    'k', 10, ...                    % sigmoid steepness
    'pings', 3, ...                 % consecutive pings required to track
    'duration', 1.0, ...            % scan duration (s)
    'scanRate', 0.2);               % time between scans (s)

% Adversaries
advConfig.count = 25;               % number of threats per trial
advConfig.speed = 15;               % UAS speed (units/s)
advConfig.turnRadius = 3;           % UAS turn radius (units)
advConfig.altitude = 10;            % UAS ingress altitude (units)
advConfig.planner = 'HybridAStar';  % path planner: 'HybridAStar' or 'Linear'

% Monte Carlo
mcSettings.maxConfigs = 5;          % number of random sensor layouts to evaluate
mcSettings.testsPerConfig = 1;      % trials per layout

% Simulation engine
simConfig.tps = 20;                 % time steps per second

% Scoring
analysisConfig.groupSize = 3;       % consecutive detections per tracking window

% Visualization
visConfig.heatmapBins = 20;         % heatmap grid resolution

%% SETUP

rng('shuffle');

mapObj = map(mapConfig.L, mapConfig.W, 1);
mapObj.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag);
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];
mapBounds = [xlims, ylims];
asset = struct('location', assetConfig.location);

trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.testsPerConfig);
configStore = cell(mcSettings.maxConfigs, 1);
scoresStore = zeros(mcSettings.maxConfigs, mcSettings.testsPerConfig);
meanScores = zeros(mcSettings.maxConfigs, 1);
allSensorLocations = [];

%% MONTE CARLO SENSOR OPTIMIZATION

for config = 1:mcSettings.maxConfigs
    sensConfig.locations = [];
    for i = 1:sensConfig.count
        valid = false;
        while ~valid
            x = rand() * mapConfig.L;
            y = rand() * mapConfig.W;
            if mapObj.getElevation(x, y) < advConfig.altitude
                if isempty(sensConfig.locations) || all(sqrt((sensConfig.locations(:,1)-x).^2 + (sensConfig.locations(:,2)-y).^2) >= sensConfig.minSpacing)
                    valid = true;
                end
            end
        end
        sensConfig.locations = [sensConfig.locations; x, y];
    end

    sensorTemplate = struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params);
    sensors = repmat(sensorTemplate, sensConfig.count, 1);
    for i = 1:sensConfig.count
        sensors(i).location = sensConfig.locations(i,:);
    end
    configStore{config} = struct('sensors', sensors, 'locations', sensConfig.locations);

    for trial = 1:mcSettings.testsPerConfig
        rng(trialSeeds(config, trial), 'twister');
        starts = ingressPosns(xlims, ylims, advConfig.count);
        uasArray = UAS.empty(0, advConfig.count);
        for k = 1:advConfig.count
            while mapObj.getElevation(starts(k,1), starts(k,2)) >= advConfig.altitude
                starts(k,:) = ingressPosns(xlims, ylims, 1);
            end
            uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, advConfig.altitude, advConfig.turnRadius);
        end

        sim = simulator(mapObj, uasArray, struct([]), sensors, asset, 'tps', simConfig.tps, 'animate', false, 'nfzs', polyshape.empty, 'resetGraphics', true);
        runResults = sim.runSim();

        allGroupProbs = [];
        for u = 1:length(runResults.detectionData)
            detData = runResults.detectionData{u};
            if isempty(detData) || size(detData, 1) < analysisConfig.groupSize; continue; end
            numGroups = floor(size(detData, 1) / analysisConfig.groupSize);
            for g = 1:numGroups
                idx = (g-1)*analysisConfig.groupSize + 1 : g*analysisConfig.groupSize;
                allGroupProbs = [allGroupProbs; prod(detData(idx, 3))];
            end
        end
        scoresStore(config, trial) = sum(allGroupProbs);
    end

    meanScores(config) = mean(scoresStore(config, :));
    allSensorLocations = [allSensorLocations; sensConfig.locations];
end

%% BEST CONFIGURATION REPLAY

[~, bestConfigIdx] = max(meanScores);
bestConfig = configStore{bestConfigIdx};

rng(trialSeeds(bestConfigIdx, 1), 'twister');
starts = ingressPosns(xlims, ylims, advConfig.count);
uasArray = UAS.empty(0, advConfig.count);
for k = 1:advConfig.count
    while mapObj.getElevation(starts(k,1), starts(k,2)) >= advConfig.altitude
        starts(k,:) = ingressPosns(xlims, ylims, 1);
    end
    uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, advConfig.altitude, advConfig.turnRadius);
end
sim = simulator(mapObj, uasArray, struct([]), bestConfig.sensors, asset, 'tps', simConfig.tps, 'animate', true, 'nfzs', polyshape.empty, 'resetGraphics', true);
sim.runSim();

%% HEATMAP

figure('Name', 'Sensor Placement Heatmap', 'Position', [100 100 900 800]);
hold on; axis equal; box on; colormap('jet');

x_edges = linspace(0, mapConfig.L, visConfig.heatmapBins+1);
y_edges = linspace(0, mapConfig.W, visConfig.heatmapBins+1);
[N, Xedges, Yedges] = histcounts2(allSensorLocations(:,1), allSensorLocations(:,2), x_edges, y_edges);
Xcenters = (Xedges(1:end-1) + Xedges(2:end))/2;
Ycenters = (Yedges(1:end-1) + Yedges(2:end))/2;
maxDensity = max(N(:));
[~, h_hm] = contourf(Xcenters, Ycenters, N', linspace(0, maxDensity, maxDensity+1), 'LineStyle', 'none');
h_hm.FaceAlpha = 0.7;
if maxDensity > 0; clim([0 maxDensity]); end
cb = colorbar; cb.Label.String = 'Sensor Placement Frequency';

plot(assetConfig.location(1), assetConfig.location(2), 'g^', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Asset');
for s = 1:sensConfig.count
    plot(bestConfig.locations(s,1), bestConfig.locations(s,2), 'ws', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Sensor %d', s));
    theta = linspace(0, 2*pi, 100);
    plot(bestConfig.locations(s,1) + sensConfig.range*cos(theta), bestConfig.locations(s,2) + sensConfig.range*sin(theta), 'w--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
xlim(xlims); ylim(ylims);
xlabel('X (units)'); ylabel('Y (units)');
title(sprintf('Sensor Placement Heatmap - Best Config #%d (Score: %.4f)', bestConfigIdx, meanScores(bestConfigIdx)));
legend('Location', 'best');
hold off;

%% SAVE

OptimizationResults.Metadata = struct( ...
    'Timestamp', datestr(now), 'MapBounds', mapBounds, ...
    'NumAdversaries', advConfig.count, 'NumSensors', sensConfig.count, ...
    'SensorRange', sensConfig.range, 'SensorParams', sensConfig.params, ...
    'AdvConfig', advConfig, 'MCConfigs', mcSettings.maxConfigs, ...
    'TestsPerConfig', mcSettings.testsPerConfig, 'GroupSize', analysisConfig.groupSize);
OptimizationResults.AllConfigurations = configStore;
OptimizationResults.AllScores = scoresStore;
OptimizationResults.MeanScores = meanScores;
OptimizationResults.BestConfigIdx = bestConfigIdx;
OptimizationResults.BestLocations = bestConfig.locations;

save(sprintf('SensorOptimization_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')), 'OptimizationResults');