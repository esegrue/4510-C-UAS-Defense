clear; clc; close all;

mapConfig.L = 100;
mapConfig.W = 100;
mapConfig.terrainType = 'Hills';
mapConfig.terrainMag = 5;

assetConfig.location = [30, 60];

sensConfig.count = 3;
sensConfig.locations = [30,85; 8.35,47.5; 51.65,47.5];
sensConfig.range = 15;
sensConfig.params = struct('d50', 25, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2);

advConfig.count = 4;
advConfig.speed = 15;
advConfig.turnRadius = 3;
advConfig.altitude = 10;
advConfig.planner = 'HybridAStar';

simConfig.tps = 20;
testConfig.numTrials = 1;

rng('shuffle')

mapBounds = [0 mapConfig.L 0 mapConfig.W];
mapObj = map(mapConfig.L, mapConfig.W, 1);
mapObj.generateTerrain(mapConfig.terrainType, mapConfig.terrainMag);
xlims = [0 mapConfig.L]; ylims = [0 mapConfig.W];

sensorStructTemplate = struct('location', [0,0], 'range', sensConfig.range, 'params', sensConfig.params);
sensors = repmat(sensorStructTemplate, sensConfig.count, 1);
for i = 1:sensConfig.count
    if i <= size(sensConfig.locations, 1)
        sensors(i).location = sensConfig.locations(i,:);
    end
end

asset = struct('location', assetConfig.location);

trialSeeds = randi([1, 2^31-1], testConfig.numTrials, 1);
allTrialResults = cell(testConfig.numTrials, 1);
allDetectionData = cell(testConfig.numTrials, 1);

for trial = 1:testConfig.numTrials
    rng(trialSeeds(trial), 'twister');
    
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
    
    allTrialResults{trial} = runResults;
    allDetectionData{trial} = runResults.detectionData;
end

SensorResults = struct();
SensorResults.Metadata = struct(...
    'Timestamp', datestr(now), ...
    'MapBounds', mapBounds, ...
    'NumAdversaries', advConfig.count, ...
    'NumSensors', sensConfig.count, ...
    'SensorLocations', sensConfig.locations, ...
    'SensorRange', sensConfig.range, ...
    'SensorParams', sensConfig.params, ...
    'AdvConfig', advConfig, ...
    'SimulationTPS', simConfig.tps ...
);

SensorResults.Sensors = sensors;
SensorResults.Asset = asset;
SensorResults.Trials = allTrialResults;
SensorResults.DetectionData = allDetectionData;
SensorResults.NumTrials = testConfig.numTrials;

for t = 1:testConfig.numTrials
    fprintf('\nTrial %d Detection Summary:\n', t);
    for u = 1:advConfig.count
        detData = allDetectionData{t}{u};
        if ~isempty(detData)
            numDetections = size(detData, 1);
            numPinged = sum(detData(:, 4));
            meanProb = mean(detData(:, 3));
            fprintf('  UAS %d: %d detections, %d pings, mean probability: %.3f\n', ...
                u, numDetections, numPinged, meanProb);
        else
            fprintf('  UAS %d: No detections\n', u);
        end
    end
end

fileName = sprintf('SensorResults_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fileName, 'SensorResults');
fprintf('\nSensor-only results saved: %s\n', fileName);