clear; clc; close all;

mapConfig.L = 100;
mapConfig.W = 100;
mapConfig.terrainType = 'Hills';
mapConfig.terrainMag = 5;

assetConfig.location = [30, 60];

sensConfig.range = 15;
sensConfig.params = struct('d50', 25, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);
sensConfig.minSpacing = 10;
sensConfig.count = 3;

advConfig.count = 50;
advConfig.speed = 15;
advConfig.turnRadius = 3;
advConfig.altitude = 10;
advConfig.planner = 'HybridAStar';

mcSettings.maxConfigs = 5;
mcSettings.testsPerConfig = 3;

simConfig.tps = 20;
simConfig.animateLive = false;
analysisConfig.groupSize = 3;
visConfig.heatmapBins = 20;

rng('shuffle')

mapBounds = [0 mapConfig.L 0 mapConfig.W];
mapObj = map(mapConfig.L, mapConfig.W, 1);
mapObj.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag);
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

asset = struct('location', assetConfig.location);

trialSeeds = randi([1, 2^31-1], mcSettings.maxConfigs, mcSettings.testsPerConfig);
configStore = cell(mcSettings.maxConfigs, 1);
scoresStore = zeros(mcSettings.maxConfigs, mcSettings.testsPerConfig);
meanScores = zeros(mcSettings.maxConfigs, 1);
allSensorLocations = [];
allConfigScores = [];

fprintf('Starting Monte Carlo Sensor Optimization...\n');
fprintf('Configurations: %d, Tests per config: %d\n\n', mcSettings.maxConfigs, mcSettings.testsPerConfig);

for config = 1:mcSettings.maxConfigs
    fprintf('Configuration %d/%d: ', config, mcSettings.maxConfigs);
    
    sensConfig.locations = [];
    for i = 1:sensConfig.count
        valid = false;
        while ~valid
            x = rand() * mapConfig.L;
            y = rand() * mapConfig.W;
            elevation = mapObj.getElevation(x, y);
            
            if elevation < advConfig.altitude
                if isempty(sensConfig.locations)
                    valid = true;
                else
                    distances = sqrt((sensConfig.locations(:,1) - x).^2 + (sensConfig.locations(:,2) - y).^2);
                    if all(distances >= sensConfig.minSpacing)
                        valid = true;
                    end
                end
            end
        end
        sensConfig.locations = [sensConfig.locations; x, y];
    end
    
    sensorStructTemplate = struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params);
    sensors = repmat(sensorStructTemplate, sensConfig.count, 1);
    for i = 1:sensConfig.count
        sensors(i).location = sensConfig.locations(i,:);
    end
    
    configStore{config} = struct('sensors', sensors, 'locations', sensConfig.locations);
    
    configScores = [];
    
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
        
        emptyEffectors = struct([]);
        
        sim = simulator(mapObj, uasArray, emptyEffectors, sensors, asset, 'tps', simConfig.tps, 'animate', false, 'nfzs', polyshape.empty, 'resetGraphics', true);
        runResults = sim.runSim();
        
        detectionData = runResults.detectionData;
        numUAS = length(detectionData);
        allGroupProbs = [];
        
        for u = 1:numUAS
            detData = detectionData{u};
            
            if isempty(detData) || size(detData, 1) < analysisConfig.groupSize
                continue;
            end
            
            numDetections = size(detData, 1);
            numGroups = floor(numDetections / analysisConfig.groupSize);
            
            for g = 1:numGroups
                startIdx = (g - 1) * analysisConfig.groupSize + 1;
                endIdx = g * analysisConfig.groupSize;
                groupProbs = detData(startIdx:endIdx, 3);
                trackingProb = prod(groupProbs);
                allGroupProbs = [allGroupProbs; trackingProb];
            end
        end
        
        overallScore = sum(allGroupProbs);
        configScores = [configScores; overallScore];
    end
    
    scoresStore(config, :) = configScores;
    meanScores(config) = mean(configScores);
    
    fprintf('Mean Score: %.4f\n', meanScores(config));
    
    allSensorLocations = [allSensorLocations; sensConfig.locations];
    allConfigScores = [allConfigScores; repmat(meanScores(config), sensConfig.count, 1)];
end

[maxScore, bestConfigIdx] = max(meanScores);
bestConfig = configStore{bestConfigIdx};
bestSensors = bestConfig.sensors;
bestLocations = bestConfig.locations;

fprintf('\n=== BEST CONFIGURATION ===\n');
fprintf('Configuration: %d\n', bestConfigIdx);
fprintf('Mean Tracking Score: %.4f\n', meanScores(bestConfigIdx));
fprintf('Sensor Locations:\n');
disp(bestLocations);

rng(trialSeeds(bestConfigIdx, 1), 'twister');
starts = ingressPosns(xlims, ylims, advConfig.count);
uasArray = UAS.empty(0, advConfig.count);
for k = 1:advConfig.count
    while mapObj.getElevation(starts(k,1), starts(k,2)) >= advConfig.altitude
        starts(k,:) = ingressPosns(xlims, ylims, 1);
    end
    uasArray(k) = UAS(advConfig.speed, starts(k,:), asset.location, advConfig.planner, advConfig.altitude, advConfig.turnRadius);
end

emptyEffectors = struct([]);
sim = simulator(mapObj, uasArray, emptyEffectors, bestSensors, asset, 'tps', simConfig.tps, 'animate', true, 'nfzs', polyshape.empty, 'resetGraphics', true);
runResults = sim.runSim();

figure('Name', 'Sensor Placement Heatmap', 'Position', [100 100 900 800]);

hold on; axis equal; box on;
colormap('jet');

x_edges = linspace(0, mapConfig.L, visConfig.heatmapBins+1);
y_edges = linspace(0, mapConfig.W, visConfig.heatmapBins+1);

[N, Xedges, Yedges] = histcounts2(allSensorLocations(:,1), allSensorLocations(:,2), x_edges, y_edges);
Xcenters = (Xedges(1:end-1) + Xedges(2:end))/2;
Ycenters = (Yedges(1:end-1) + Yedges(2:end))/2;

maxDensity = max(N(:));
levels = linspace(0, maxDensity, maxDensity+1);

[C_hm, h_hm] = contourf(Xcenters, Ycenters, N', levels, 'LineStyle', 'none');
h_hm.FaceAlpha = 0.7;

if maxDensity > 0
    clim([0 maxDensity]);
end

cb = colorbar;
cb.Label.String = 'Sensor Placement Frequency';

plot(assetConfig.location(1), assetConfig.location(2), 'g^', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Asset');

for s = 1:sensConfig.count
    plot(bestLocations(s, 1), bestLocations(s, 2), 'ws', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Best Sensor %d', s));
    theta = linspace(0, 2*pi, 100);
    sensorCircle_x = bestLocations(s, 1) + sensConfig.range * cos(theta);
    sensorCircle_y = bestLocations(s, 2) + sensConfig.range * sin(theta);
    plot(sensorCircle_x, sensorCircle_y, 'w--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

xlim(xlims); ylim(ylims);
xlabel('X Coordinate (units)'); ylabel('Y Coordinate (units)');
title(sprintf('Sensor Placement Heatmap - Best Config #%d (Score: %.4f)', bestConfigIdx, maxScore));
legend('Location', 'best');

hold off;

OptimizationResults = struct();
OptimizationResults.Metadata = struct(...
    'Timestamp', datestr(now), ...
    'MapBounds', mapBounds, ...
    'NumAdversaries', advConfig.count, ...
    'NumSensors', sensConfig.count, ...
    'SensorRange', sensConfig.range, ...
    'SensorParams', sensConfig.params, ...
    'AdvConfig', advConfig, ...
    'MCConfigs', mcSettings.maxConfigs, ...
    'TestsPerConfig', mcSettings.testsPerConfig, ...
    'GroupSize', analysisConfig.groupSize ...
);

OptimizationResults.AllConfigurations = configStore;
OptimizationResults.AllScores = scoresStore;
OptimizationResults.MeanScores = meanScores;
OptimizationResults.BestConfigIdx = bestConfigIdx;
OptimizationResults.BestLocations = bestLocations;

fileName = sprintf('SensorOptimization_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fileName, 'OptimizationResults');
fprintf('\nOptimization results saved: %s\n', fileName);