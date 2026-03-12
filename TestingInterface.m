clear;
clc;
close all;


%% NOTES 

% Do not plot every single time (wastes computing power) 
% Merge with new Adversary and Map code 



%% CODE

DEFENDER_MODE = "MOBILE"; % defender motion mode ("STATIC" or "MOBILE")
GRID2D = false; % enable 2-D grid plot

SHOW_FINAL_REPLAY = true; % replay best configuration at end
SHOW_MC_FIGURES = false; % show a figure for every Monte Carlo run
PRINT_EACH_RUN = false; % print each Monte Carlo run line

NUM_ADVERSARIES = 1; % number of adversaries

ADVERSARY_MODE = "LINEAR"; % adversary motion mode ("LINEAR" or "HYBRIDASTAR")
ADVERSARY_EDGES = "ALL"; % adversary ingress edges ("ALL", "BOTTOM", "RIGHT", "TOP", "LEFT")
ADVERSARY_ALTITUDE = 25; % adversary altitude (m)
ADVERSARY_SPEED = 15; % adversary speed (m/s)

DEF_TRIANGLE_RADIUS = 10; % defender distance from asset (m)
DEF_TRIANGLE_ORIENTATION_DEG = 0; % triangle rotation (deg), 0 points one vertex toward +X
DEFENDER_SPEED = 12; % defender speed when mobile (m/s)

DEFENDER_WEAPON_MODE = "KINETIC"; % "DIRECT_ENERGY", "KINETIC", "KINECT", or "LEGACY"
DIRECT_ENERGY_DWELL_TIME = 0.75; % required dwell time inside effector range (s)

KINETIC_PROJECTILE_SPEED = 30; % projectile speed (m/s)
KINETIC_SHOTS_PER_VOLLEY = 3; % projectiles fired per engagement
KINETIC_HIT_PROBABILITY = 1.0; % probability each projectile is a hit
KINETIC_REFIRE_TIME = 0.50; % min time between volleys from the same effector (s)
PROJECTILE_HIT_TOLERANCE = 1.0; % projectile-to-adversary hit tolerance (m)

ANIM_DRAW_EVERY = 1; % draw every N ticks
ANIM_MULTIPLIER = 0.7; % animation playback multiplier
ANIM_TPS = 20; % ticks per second
ANIM_MAX_SIM_TIME = 20; % max sim time for previews (s)

maxConfigs = 100; % max Monte Carlo configurations
num_tests = 10; % runs per configuration
delta = 100; % separation threshold for convergence

previewAfterConfigs = 1; % first config index to allow preview
previewEveryConfigs = 10; % preview cadence (configs)
previewMaxRuns = 1; % max runs to replay per preview
lastPreviewConfig = 0; % last previewed config index

configStore = cell(maxConfigs, 1); % effector configurations per config
scenarioStore = cell(maxConfigs, 1); % adversary start scenarios per config
costDetailsStore = cell(maxConfigs, 1); % run costs per config

CostperCombo = nan(maxConfigs, 1); % mean cost per config
ReliabilityScore = nan(maxConfigs, 1); % reliability percentage per config
LCB = nan(maxConfigs, 1); % lower confidence bound for cost
UCB = nan(maxConfigs, 1); % upper confidence bound for cost
separation = nan(maxConfigs, 1); % best-vs-rival separation metric

killStore = cell(maxConfigs, 1); % kill locations per config

cfgIdx = (1:maxConfigs)'; % config indices

figMC = figure('Name', 'Monte Carlo Metrics', 'NumberTitle', 'off');
clf(figMC);

ax1 = subplot(3, 1, 1, 'Parent', figMC);
hCost = plot(ax1, nan, nan, '-o');
grid(ax1, 'on');
box(ax1, 'on');
ylabel(ax1, 'Mean Cost');
title(ax1, 'Monte Carlo Metrics');
xlim(ax1, [1 maxConfigs]);

ax2 = subplot(3, 1, 2, 'Parent', figMC);
hRel = plot(ax2, nan, nan, '-o');
grid(ax2, 'on');
box(ax2, 'on');
ylabel(ax2, 'Reliability (%)');
xlim(ax2, [1 maxConfigs]);

ax3 = subplot(3, 1, 3, 'Parent', figMC);
hSep = plot(ax3, nan, nan, '-o');
grid(ax3, 'on');
box(ax3, 'on');
ylabel(ax3, 'Separation');
xlabel(ax3, 'Config #');
xlim(ax3, [1 maxConfigs]);
yline(ax3, delta, '--');

drawnow;

mapL = 100; % map length (m)
mapW = 100; % map width (m)
mapObj = map(mapL, mapW, 1); % map object (resolution = 1)
mapObj.generateTerrain('Hills', 30); % generate terrain
xlims = [0 mapL]; % map x-limits
ylims = [0 mapW]; % map y-limits

numAssets = 1; % number of assets
numSensors = 1; % number of sensors
numEffectors = 3; % number of effectors attached to defenders

matCandidates = ["effector_posns_bank.mat", "effector_posn_bank.mat"]; % allowed MAT filenames
matFile = ""; % selected MAT filename

for ii = 1:numel(matCandidates)
    if isfile(matCandidates(ii))
        matFile = matCandidates(ii);
        break
    end
end

if matFile == ""
    error("Could not find effector position bank MAT-file. Expected one of: %s", strjoin(matCandidates, ", "));
end

bankLoaded = load(matFile);

if isfield(bankLoaded, "effector_posns_bank")
    effector_posns_bank = bankLoaded.effector_posns_bank;
elseif isfield(bankLoaded, "effector_posn_bank")
    effector_posns_bank = bankLoaded.effector_posn_bank;
else
    error("Loaded '%s' but could not find variable 'effector_posns_bank' or 'effector_posn_bank' inside it.", matFile);
end

bankCount = getEffectorBankCount(effector_posns_bank, numEffectors);
if bankCount < 1
    error("Effector bank appears empty or incompatible with numEffectors=%d.", numEffectors);
end

assetStructTemplate = struct('location', [0, 0]); % asset struct template
assets = repmat(assetStructTemplate, numAssets, 1); % asset array
assets(1).location = [30, 60]; % asset location [x y]

sensorRange = 15; % sensor detection range (m)
effectorRange = 12; % effector intercept range (m)

params = struct('d50', sensorRange, 'k', 10, 'pings', 3, 'duration', 1.0, 'scanRate', 0.2); % sensor detection parameters
sensorStructTemplate = struct('location', [0, 0], 'range', sensorRange, 'params', params); % sensor struct template
sensors = repmat(sensorStructTemplate, numSensors, 1); % sensor array
sensors(1).location = assets(1).location; % colocated sensor

effectorStructTemplate = struct('location', [0, 0], 'range', effectorRange, 'defenderIdx', 1); % effector struct template

AORo = assets(1).location; % AOR center
AORsize = 60; % AOR side length
AOR = polyshape([AORo(1) - AORsize / 2, AORo(1) - AORsize / 2, AORo(1) + AORsize / 2, AORo(1) + AORsize / 2], ...
                [AORo(2) + AORsize / 2, AORo(2) - AORsize / 2, AORo(2) - AORsize / 2, AORo(2) + AORsize / 2]);

NFZbaseVerts = [18 70; 25 88; 33 72; 42 90; 38 66; 28 68]; % base NFZ polygon vertices
NFZKeepOut = 1.0; % NFZ buffer distance (m)
NFZMarginFromMapEdge = 5.0; % minimum distance from map edge (m)
NFZMaxTries = 600; % max NFZ randomization attempts

costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250); % cost model
reliabilityThreshold = 90; % minimum reliability percentage for valid configs

axes(ax2); 
hold(ax2, 'on');
yline(ax2, reliabilityThreshold, '--');
hold(ax2, 'off');

fprintf('\nSTARTUP PREVIEW: Defenders+Effectors start from MAT bank and move together...\n');

rng(1);
starts0 = ingressPosns(xlims, ylims, NUM_ADVERSARIES, ADVERSARY_EDGES);

NFZ0 = randomizeNFZ(NFZbaseVerts, xlims, ylims, NFZMarginFromMapEdge, NFZMaxTries);
NFZ0 = bufferPolySafe(NFZ0, NFZKeepOut);

previewConfigIdx = 1;
effXY0 = sampleEffectorConfigXY(effector_posns_bank, numEffectors, previewConfigIdx);

defenders0 = makeDefendersFromEffectorXY(effXY0, DEFENDER_MODE, DEFENDER_SPEED);

currentEffectors0 = repmat(effectorStructTemplate, numEffectors, 1);
for e = 1:numEffectors
    currentEffectors0(e).defenderIdx = e;
    currentEffectors0(e).location = defenders0(e).location;
end

uasArray0 = UAS.empty(0, 1);
for k = 1:NUM_ADVERSARIES
    uasArray0(k) = UAS(ADVERSARY_SPEED, starts0(k, :), assets(1).location, ADVERSARY_MODE, ADVERSARY_ALTITUDE);
end

simStartup = simulator(mapObj, AOR, uasArray0, currentEffectors0, sensors, assets, ...
    'tps', ANIM_TPS, ...
    'animate', true, ...
    'fadePings', true, ...
    'nfzs', NFZ0, ...
    'resetGraphics', true, ...
    'animationMultiplier', ANIM_MULTIPLIER, ...
    'costConfig', costConfig, ...
    'defenders', defenders0, ...
    'grid2D', GRID2D, ...
    'drawEvery', ANIM_DRAW_EVERY, ...
    'maxSimTime', ANIM_MAX_SIM_TIME, ...
    'weaponMode', DEFENDER_WEAPON_MODE, ...
    'directEnergyDwellTime', DIRECT_ENERGY_DWELL_TIME, ...
    'kineticProjectileSpeed', KINETIC_PROJECTILE_SPEED, ...
    'kineticShotsPerVolley', KINETIC_SHOTS_PER_VOLLEY, ...
    'kineticHitProbability', KINETIC_HIT_PROBABILITY, ...
    'kineticRefireTime', KINETIC_REFIRE_TIME, ...
    'projectileHitTolerance', PROJECTILE_HIT_TOLERANCE);

simStartup.runSim();
drawnow;
shg;

numConfigs = 0;

while numConfigs < maxConfigs
    numConfigs = numConfigs + 1;

    fprintf('\n====================================================\n');
    fprintf('CONFIG (ITERATION) %d / %d\n', numConfigs, maxConfigs);
    fprintf('====================================================\n');

    effXY = sampleEffectorConfigXY(effector_posns_bank, numEffectors, numConfigs);

    defenders = makeDefendersFromEffectorXY(effXY, DEFENDER_MODE, DEFENDER_SPEED);

    currentEffectors = repmat(effectorStructTemplate, numEffectors, 1);
    for e = 1:numEffectors
        currentEffectors(e).defenderIdx = e;
        currentEffectors(e).location = defenders(e).location;
    end

    configStore{numConfigs} = currentEffectors;

    runCosts = zeros(num_tests, 1);
    runStarts = cell(num_tests, 1);
    runFailures = 0;
    currentConfigKills = [];

    for j = 1:num_tests
        runSeed = numConfigs * 1000 + j;
        if PRINT_EACH_RUN
            fprintf('  Run %d/%d (seed=%d)\n', j, num_tests, runSeed);
        end
        rng(runSeed);

        starts = ingressPosns(xlims, ylims, NUM_ADVERSARIES, ADVERSARY_EDGES);

        NFZ = randomizeNFZ(NFZbaseVerts, xlims, ylims, NFZMarginFromMapEdge, NFZMaxTries);
        NFZ = bufferPolySafe(NFZ, NFZKeepOut);

        uasArray = UAS.empty(0, 1);
        for k = 1:NUM_ADVERSARIES
            uasArray(k) = UAS(ADVERSARY_SPEED, starts(k, :), assets(1).location, ADVERSARY_MODE, ADVERSARY_ALTITUDE);
        end

        sim = simulator(mapObj, AOR, uasArray, currentEffectors, sensors, assets, ...
            'tps', ANIM_TPS, ...
            'animate', false, ...
            'showFigure', SHOW_MC_FIGURES, ...
            'nfzs', NFZ, ...
            'resetGraphics', true, ...
            'costConfig', costConfig, ...
            'defenders', defenders, ...
            'grid2D', GRID2D, ...
            'weaponMode', DEFENDER_WEAPON_MODE, ...
            'directEnergyDwellTime', DIRECT_ENERGY_DWELL_TIME, ...
            'kineticProjectileSpeed', KINETIC_PROJECTILE_SPEED, ...
            'kineticShotsPerVolley', KINETIC_SHOTS_PER_VOLLEY, ...
            'kineticHitProbability', KINETIC_HIT_PROBABILITY, ...
            'kineticRefireTime', KINETIC_REFIRE_TIME, ...
            'projectileHitTolerance', PROJECTILE_HIT_TOLERANCE);

        runResults = sim.runSim();
        runCosts(j) = runResults.cost;
        runStarts{j} = starts;

        if isfield(runResults, 'UASkillLocations') && ~isempty(runResults.UASkillLocations)
            currentConfigKills = [currentConfigKills; runResults.UASkillLocations]; %#ok<AGROW>
        end

        if runResults.cost >= costConfig.asset
            runFailures = runFailures + 1;
        end
    end

    scenarioStore{numConfigs} = runStarts;
    costDetailsStore{numConfigs} = runCosts;
    CostperCombo(numConfigs) = mean(runCosts);
    killStore{numConfigs} = currentConfigKills;

    currentReliability = 100 * (1 - (runFailures / num_tests));
    ReliabilityScore(numConfigs) = currentReliability;

    SD = std(runCosts, 0);
    SE = SD / sqrt(num_tests);
    alpha = 0.1;
    tcrit = tinv(1 - alpha / 2, num_tests - 1);
    LCB(numConfigs) = CostperCombo(numConfigs) - tcrit * SE;
    UCB(numConfigs) = CostperCombo(numConfigs) + tcrit * SE;

    validMask = find(ReliabilityScore(1:numConfigs) >= reliabilityThreshold);

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
            numConfigs, bestGlobalID, CostperCombo(bestGlobalID), min(CostperCombo(rivalsMask)), separation(numConfigs));
    end

    if isgraphics(figMC)
        x = cfgIdx(1:numConfigs);
        set(hCost, 'XData', x, 'YData', CostperCombo(1:numConfigs));
        set(hRel, 'XData', x, 'YData', ReliabilityScore(1:numConfigs));
        set(hSep, 'XData', x, 'YData', separation(1:numConfigs));

        xlim(ax1, [1 max(2, numConfigs)]);
        xlim(ax2, [1 max(2, numConfigs)]);
        xlim(ax3, [1 max(2, numConfigs)]);

        drawnow limitrate;
    end

    doPreview = false;
    if numConfigs >= previewAfterConfigs && lastPreviewConfig == 0
        doPreview = true;
    elseif previewEveryConfigs > 0 && mod(numConfigs, previewEveryConfigs) == 0 && lastPreviewConfig ~= numConfigs
        doPreview = true;
    end

    if doPreview
        fprintf('\nPREVIEW: Animating best-so-far (defenders+effectors move together)...\n');

        validCandidates = find(ReliabilityScore(1:numConfigs) >= reliabilityThreshold);
        if isempty(validCandidates)
            [~, previewID] = max(ReliabilityScore(1:numConfigs));
        else
            [~, idxp] = min(CostperCombo(validCandidates));
            previewID = validCandidates(idxp);
        end

        scenariosToReplay = scenarioStore{previewID};
        previewEffectors = configStore{previewID};
        previewRuns = min(previewMaxRuns, numel(scenariosToReplay));

        effXYp = sampleEffectorConfigXY(effector_posns_bank, numEffectors, previewID);
        defendersP = makeDefendersFromEffectorXY(effXYp, DEFENDER_MODE, DEFENDER_SPEED);

        for kk = 1:previewRuns
            replaySeed = previewID * 1000 + kk;
            rng(replaySeed);

            replayStartPos = scenariosToReplay{kk};

            NFZ = randomizeNFZ(NFZbaseVerts, xlims, ylims, NFZMarginFromMapEdge, NFZMaxTries);
            NFZ = bufferPolySafe(NFZ, NFZKeepOut);

            uasArray = UAS.empty(0, 1);
            for u = 1:NUM_ADVERSARIES
                uasArray(u) = UAS(ADVERSARY_SPEED, replayStartPos(u, :), assets(1).location, ADVERSARY_MODE, ADVERSARY_ALTITUDE);
            end

            simPreview = simulator(mapObj, AOR, uasArray, previewEffectors, sensors, assets, ...
                'tps', ANIM_TPS, ...
                'animate', true, ...
                'fadePings', true, ...
                'nfzs', NFZ, ...
                'resetGraphics', true, ...
                'animationMultiplier', ANIM_MULTIPLIER, ...
                'costConfig', costConfig, ...
                'defenders', defendersP, ...
                'grid2D', GRID2D, ...
                'drawEvery', ANIM_DRAW_EVERY, ...
                'maxSimTime', ANIM_MAX_SIM_TIME, ...
                'weaponMode', DEFENDER_WEAPON_MODE, ...
                'directEnergyDwellTime', DIRECT_ENERGY_DWELL_TIME, ...
                'kineticProjectileSpeed', KINETIC_PROJECTILE_SPEED, ...
                'kineticShotsPerVolley', KINETIC_SHOTS_PER_VOLLEY, ...
                'kineticHitProbability', KINETIC_HIT_PROBABILITY, ...
                'kineticRefireTime', KINETIC_REFIRE_TIME, ...
                'projectileHitTolerance', PROJECTILE_HIT_TOLERANCE);

            simPreview.runSim();
            drawnow;
            shg;
        end

        lastPreviewConfig = numConfigs;
    end

    if separation(numConfigs) > delta
        fprintf('Monte Carlo Simulation converged!\n');
        break
    end
end

if SHOW_FINAL_REPLAY
    fprintf('\nFINAL REPLAY: Animating best configuration found (defenders+effectors move together)...\n');

    validCandidates = find(ReliabilityScore(1:numConfigs) >= reliabilityThreshold);
    if isempty(validCandidates)
        [~, bestID] = max(ReliabilityScore(1:numConfigs));
    else
        [~, idxb] = min(CostperCombo(validCandidates));
        bestID = validCandidates(idxb);
    end

    scenariosToReplay = scenarioStore{bestID};
    bestEffectors = configStore{bestID};

    effXYb = sampleEffectorConfigXY(effector_posns_bank, numEffectors, bestID);
    defendersB = makeDefendersFromEffectorXY(effXYb, DEFENDER_MODE, DEFENDER_SPEED);

    if ~isempty(scenariosToReplay)
        replaySeed = bestID * 1000 + 1;
        rng(replaySeed);

        replayStartPos = scenariosToReplay{1};

        NFZ = randomizeNFZ(NFZbaseVerts, xlims, ylims, NFZMarginFromMapEdge, NFZMaxTries);
        NFZ = bufferPolySafe(NFZ, NFZKeepOut);

        uasArray = UAS.empty(0, 1);
        for u = 1:NUM_ADVERSARIES
            uasArray(u) = UAS(ADVERSARY_SPEED, replayStartPos(u, :), assets(1).location, ADVERSARY_MODE, ADVERSARY_ALTITUDE);
        end

        simFinal = simulator(mapObj, AOR, uasArray, bestEffectors, sensors, assets, ...
            'tps', ANIM_TPS, ...
            'animate', true, ...
            'fadePings', true, ...
            'nfzs', NFZ, ...
            'resetGraphics', true, ...
            'animationMultiplier', ANIM_MULTIPLIER, ...
            'costConfig', costConfig, ...
            'defenders', defendersB, ...
            'grid2D', GRID2D, ...
            'drawEvery', ANIM_DRAW_EVERY, ...
            'maxSimTime', ANIM_MAX_SIM_TIME, ...
            'weaponMode', DEFENDER_WEAPON_MODE, ...
            'directEnergyDwellTime', DIRECT_ENERGY_DWELL_TIME, ...
            'kineticProjectileSpeed', KINETIC_PROJECTILE_SPEED, ...
            'kineticShotsPerVolley', KINETIC_SHOTS_PER_VOLLEY, ...
            'kineticHitProbability', KINETIC_HIT_PROBABILITY, ...
            'kineticRefireTime', KINETIC_REFIRE_TIME, ...
            'projectileHitTolerance', PROJECTILE_HIT_TOLERANCE);

        simFinal.runSim();
        drawnow;
        shg;
    end
end

function defenders = makeDefendersFromEffectorXY(effXY, mode, speed)
    n = size(effXY, 1);
    defTemplate = struct( ...
        'location', [0 0], ...
        'speed', speed, ...
        'mode', string(mode), ...
        'heading', 0, ...
        'planner', [], ...
        'path', [], ...
        'pathIdx', 1, ...
        'lastPlanTick', -inf, ...
        'lastInterceptPose', [nan nan nan]);

    defenders = repmat(defTemplate, n, 1);

    for i = 1:n
        defenders(i).location = effXY(i, 1:2);
        defenders(i).heading = 0;
        if defenders(i).mode == "STATIC"
            defenders(i).speed = 0;
        end
    end
end

function n = getEffectorBankCount(bank, numEffectors)
    n = 0;

    if isnumeric(bank)
        if ndims(bank) == 3 && size(bank, 2) == 2 && size(bank, 3) == numEffectors
            n = size(bank, 1);
            return
        end
        if ismatrix(bank) && size(bank, 2) == 2 * numEffectors
            n = size(bank, 1);
            return
        end
        if ismatrix(bank) && size(bank, 2) == 2
            n = floor(size(bank, 1) / numEffectors);
            return
        end
    end

    if iscell(bank)
        n = numel(bank);
        return
    end

    if isstruct(bank)
        n = numel(bank);
        return
    end
end

function effXY = sampleEffectorConfigXY(bank, numEffectors, configIdx)
    bankCount = getEffectorBankCount(bank, numEffectors);
    if bankCount < 1
        error("Effector bank incompatible with numEffectors=%d.", numEffectors);
    end

    idx = mod(configIdx - 1, bankCount) + 1;

    if isnumeric(bank)
        if ndims(bank) == 3 && size(bank, 2) == 2 && size(bank, 3) == numEffectors
            effXY = squeeze(bank(idx, :, :))';
            return
        end
        if ismatrix(bank) && size(bank, 2) == 2 * numEffectors
            row = bank(idx, :);
            effXY = reshape(row, 2, numEffectors)';
            return
        end
        if ismatrix(bank) && size(bank, 2) == 2
            i0 = (idx - 1) * numEffectors + 1;
            i1 = i0 + numEffectors - 1;
            if i1 > size(bank, 1)
                error("Not enough rows in N x 2 bank for config %d and numEffectors=%d.", idx, numEffectors);
            end
            effXY = bank(i0:i1, 1:2);
            return
        end
    end

    if iscell(bank)
        cfg = bank{idx};
        effXY = sampleEffectorConfigXY(cfg, numEffectors, 1);
        return
    end

    if isstruct(bank)
        cfg = bank(idx);
        if isfield(cfg, "posns")
            effXY = sampleEffectorConfigXY(cfg.posns, numEffectors, 1);
            return
        end
        if isfield(cfg, "positions")
            effXY = sampleEffectorConfigXY(cfg.positions, numEffectors, 1);
            return
        end
        if isfield(cfg, "xy")
            effXY = sampleEffectorConfigXY(cfg.xy, numEffectors, 1);
            return
        end
        error("Struct bank config does not contain posns/positions/xy fields.");
    end

    error("Unsupported effector bank format.");
end

function v = cleanPolyVerts(v, dupTol, colTol)
    keep = true(size(v, 1), 1);
    for i = 2:size(v, 1)
        if norm(v(i, :) - v(i - 1, :)) < dupTol
            keep(i) = false;
        end
    end
    v = v(keep, :);

    if size(v, 1) >= 2 && norm(v(end, :) - v(1, :)) < dupTol
        v(end, :) = [];
    end

    changed = true;
    while changed && size(v, 1) >= 3
        changed = false;
        n = size(v, 1);
        keep = true(n, 1);
        for i = 1:n
            i0 = mod(i - 2, n) + 1;
            i1 = i;
            i2 = mod(i, n) + 1;

            a = v(i1, :) - v(i0, :);
            b = v(i2, :) - v(i1, :);

            if norm(a) < dupTol || norm(b) < dupTol
                keep(i1) = false;
                changed = true;
                continue
            end

            crossz = abs(a(1) * b(2) - a(2) * b(1));
            if crossz < colTol
                keep(i1) = false;
                changed = true;
            end
        end
        v = v(keep, :);
    end
end

function NFZ = randomizeNFZ(baseVerts, mapXLim, mapYLim, margin, maxTries)
    c = mean(baseVerts, 1);
    base0 = baseVerts - c;

    for k = 1:maxTries
        ang = (2 * rand - 1) * pi;
        R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
        vr = (R * base0')';

        minX = min(vr(:, 1));
        maxX = max(vr(:, 1));
        minY = min(vr(:, 2));
        maxY = max(vr(:, 2));

        txMin = mapXLim(1) + margin - minX;
        txMax = mapXLim(2) - margin - maxX;
        tyMin = mapYLim(1) + margin - minY;
        tyMax = mapYLim(2) - margin - maxY;

        if txMin > txMax || tyMin > tyMax
            continue
        end

        tx = txMin + rand() * (txMax - txMin);
        ty = tyMin + rand() * (tyMax - tyMin);

        v = vr + [tx ty];
        v = cleanPolyVerts(v, 1e-4, 1e-3);
        if size(v, 1) < 3
            continue
        end

        lastwarn("");
        NFZcand = polyshape(v(:, 1), v(:, 2), "Simplify", true);
        [~, wid] = lastwarn();
        if ~isempty(wid)
            continue
        end

        if NFZcand.NumRegions == 1 && area(NFZcand) > 1e-6
            NFZ = NFZcand;
            return
        end
    end

    NFZ = polyshape(baseVerts(:, 1), baseVerts(:, 2), "Simplify", true);
end

function P = bufferPolySafe(Pin, r)
    P = Pin;
    if r <= 0
        return
    end

    try
        Pbuf = polybuffer(Pin, r);
        if Pbuf.NumRegions >= 1 && area(Pbuf) > 1e-6
            P = Pbuf;
        end
    catch
        P = Pin;
    end
end
