clear; clc; close all;

%% CONFIGURATION

% Map
mapConfig.L = 100;
mapConfig.W = 100;
mapConfig.terrainType = 'Hills';
mapConfig.terrainMag = 5;

% Asset — centered on map
assetConfig.location = [50, 50];

% Sensors
sensConfig.count = 3;
sensConfig.range = 15;
sensConfig.minSpacing = 10;
sensConfig.params = struct( ...
    'd50', 25, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

% Adversaries
advConfig.count = 25;
advConfig.speed = 15;
advConfig.turnRadius = 5;
advConfig.altitude = 10;
advConfig.planner = 'HybridAStar';

% Monte Carlo
mcSettings.maxConfigs = 10;
mcSettings.testsPerConfig = 1;

% Simulation engine
simConfig.tps = 20;

% Scoring
analysisConfig.groupSize = 3;

% Visualization
visConfig.heatmapBins = 20;

%% SETUP

rng('shuffle');

% Ensure process-based parallel pool
pool = gcp('nocreate');
if ~isempty(pool) && isa(pool, 'parallel.ThreadPool')
    delete(pool);
end
if isempty(gcp('nocreate'))
    parpool('Processes');
end

mapObj = map(mapConfig.L, mapConfig.W, 1);
mapObj.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag);
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];
mapBounds = [xlims, ylims];
asset = struct('location', assetConfig.location);

% No-Fly Zones — centered around asset at [50, 50]
nfzArray = [ ...
    polyshape([25 35 35 25],   [40 40 55 55]);       % left of asset
    polyshape([60 72 72 60],   [45 45 58 58]);       % right of asset
    polyshape([40 50 53 45 38],[65 63 72 78 74]);     % above asset
    polyshape([42 54 54 42],   [28 28 40 40]);       % below asset
    polyshape([75 87 90 78],   [10 10 22 22]);       % bottom-right corner
    polyshape([70 83 80 67],   [75 73 87 87]);       % top-right corner
    polyshape([5  20 18  5],   [70 70 85 85]);       % top-left corner
];

trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.testsPerConfig);
configStore = cell(mcSettings.maxConfigs, 1);
meanScores  = zeros(mcSettings.maxConfigs, 1);
allLocs     = cell(mcSettings.maxConfigs, 1);

x_edges = linspace(0, mapConfig.L, visConfig.heatmapBins+1);
y_edges = linspace(0, mapConfig.W, visConfig.heatmapBins+1);

%% MONTE CARLO SENSOR OPTIMIZATION (PARALLEL)

parfor config = 1:mcSettings.maxConfigs
    localMap = map(mapConfig.L, mapConfig.W, 1);
    localMap.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag);

    locs = zeros(sensConfig.count, 2);
    for i = 1:sensConfig.count
        valid = false;
        while ~valid
            pt = [rand()*mapConfig.L, rand()*mapConfig.W];
            belowAlt = localMap.getElevation(pt(1), pt(2)) < advConfig.altitude;
            inNFZ = false;
            for z = 1:length(nfzArray)
                if isinterior(nfzArray(z), pt(1), pt(2))
                    inNFZ = true; break;
                end
            end
            if i == 1
                spaced = true;
            else
                spaced = all(vecnorm(locs(1:i-1,:) - pt, 2, 2) >= sensConfig.minSpacing);
            end
            valid = belowAlt && spaced && ~inNFZ;
        end
        locs(i,:) = pt;
    end

    sensors = repmat(struct('location',[0,0],'range',sensConfig.range,'params',sensConfig.params), sensConfig.count, 1);
    for i = 1:sensConfig.count
        sensors(i).location = locs(i,:);
    end
    configStore{config} = struct('sensors', sensors, 'locations', locs);

    trialScores = zeros(1, mcSettings.testsPerConfig);
    for trial = 1:mcSettings.testsPerConfig
        rng(trialSeeds(config, trial), 'twister');
        starts = ingressPosns(xlims, ylims, advConfig.count);
        uasArray = UAS.empty(0, advConfig.count);
        for k = 1:advConfig.count
            while localMap.getElevation(starts(k,1), starts(k,2)) >= advConfig.altitude
                starts(k,:) = ingressPosns(xlims, ylims, 1);
            end
            uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, advConfig.altitude, advConfig.turnRadius);
        end

        sim = simulator(localMap, uasArray, struct([]), sensors, asset, ...
            'tps', simConfig.tps, 'animate', false, 'nfzs', nfzArray, 'resetGraphics', true);
        runResults = sim.runSim();

        score = 0;
        for u = 1:length(runResults.detectionData)
            detData = runResults.detectionData{u};
            if isempty(detData) || size(detData,1) < analysisConfig.groupSize; continue; end
            numGroups = floor(size(detData,1) / analysisConfig.groupSize);
            for g = 1:numGroups
                idx = (g-1)*analysisConfig.groupSize + 1 : g*analysisConfig.groupSize;
                score = score + prod(detData(idx, 3));
            end
        end
        trialScores(trial) = score;
    end
    meanScores(config) = mean(trialScores);
    allLocs{config} = locs;
end

%% BUILD HEATMAP DATA

scoreSum   = zeros(visConfig.heatmapBins);
scoreCount = zeros(visConfig.heatmapBins);

for config = 1:mcSettings.maxConfigs
    locs = allLocs{config};
    for i = 1:sensConfig.count
        col = find(locs(i,1) >= x_edges(1:end-1) & locs(i,1) < x_edges(2:end), 1);
        row = find(locs(i,2) >= y_edges(1:end-1) & locs(i,2) < y_edges(2:end), 1);
        if ~isempty(col) && ~isempty(row)
            scoreSum(col, row)  = scoreSum(col, row) + meanScores(config);
            scoreCount(col, row) = scoreCount(col, row) + 1;
        end
    end
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
sim = simulator(mapObj, uasArray, struct([]), bestConfig.sensors, asset, ...
    'tps', simConfig.tps, 'animate', true, 'nfzs', nfzArray, 'resetGraphics', true);
sim.runSim();

%% HEATMAP

avgScore = zeros(size(scoreSum));
mask = scoreCount > 0;
avgScore(mask) = scoreSum(mask) ./ scoreCount(mask);

Xcenters = (x_edges(1:end-1) + x_edges(2:end)) / 2;
Ycenters = (y_edges(1:end-1) + y_edges(2:end)) / 2;

figure('Name','Sensor Performance Heatmap');
imagesc(Xcenters, Ycenters, avgScore');
axis xy equal tight; colormap('jet'); colorbar;
hold on;
for n = 1:length(nfzArray)
    plot(nfzArray(n), 'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.6, 'EdgeColor', 'w', 'LineWidth', 1.5);
end
plot(assetConfig.location(1), assetConfig.location(2), 'g^', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Asset');
for s = 1:sensConfig.count
    plot(bestConfig.locations(s,1), bestConfig.locations(s,2), 'ws', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Sensor %d', s));
end
xlabel('X'); ylabel('Y');
title(sprintf('Avg Detection Score by Cell — Best: Config #%d (%.2f)', bestConfigIdx, meanScores(bestConfigIdx)));
legend('Location','best');
hold off;

%% SAVE

OptimizationResults.Metadata = struct( ...
    'Timestamp', datestr(now), 'MapBounds', mapBounds, ...
    'NumAdversaries', advConfig.count, 'NumSensors', sensConfig.count, ...
    'SensorRange', sensConfig.range, 'SensorParams', sensConfig.params, ...
    'AdvConfig', advConfig, 'MCConfigs', mcSettings.maxConfigs, ...
    'TestsPerConfig', mcSettings.testsPerConfig, 'GroupSize', analysisConfig.groupSize);
OptimizationResults.AllConfigurations = configStore;
OptimizationResults.MeanScores = meanScores;
OptimizationResults.BestConfigIdx = bestConfigIdx;
OptimizationResults.BestLocations = bestConfig.locations;
OptimizationResults.NFZs = nfzArray;

save(sprintf('SensorOptimization_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')), 'OptimizationResults');