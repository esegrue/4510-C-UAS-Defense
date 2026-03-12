classdef map < handle
    properties
        size
        terrain
        resolution
        terrainProxy

        UASTrail
        UASHead
        UASsensed
        UASkilled
        UAScrashed
        assetDestroyed
        assets
        timeBox

        NFZhandle
        obstacleHandles
        defenderHandles

        effectorHandles
        effectorRingHandles
        sensorHandles
        sensorRingHandles

        figSim
        axSim
        figTerr
        axTerr

        maxTrailPoints (1, 1) double = 400

        occMapCache
        occMapCacheThreshold (1, 1) double = NaN

        eventBox

        defenderTrailHandles
        defenderTrailHist
        defenderMaxTrailPoints (1, 1) double = 400

        projectileHandles

        killFlashHandles
        killFlashExpireTimes
        killFlashDuration (1, 1) double = 0.35

        is3DView (1, 1) logical = false
    end

    methods
        function obj = map(vertical, horizontal, resolution)
            if nargin < 3
                resolution = 1;
            end

            obj.size.vert = vertical;
            obj.size.horiz = horizontal;
            obj.resolution = resolution;
            obj.generateTerrain('Flat');

            obj.occMapCache = [];
            obj.occMapCacheThreshold = NaN;
            obj.eventBox = [];

            obj.figSim = [];
            obj.axSim = [];
            obj.figTerr = [];
            obj.axTerr = [];

            obj.defenderTrailHandles = gobjects(0);
            obj.defenderTrailHist = {};
            obj.projectileHandles = gobjects(0);

            obj.killFlashHandles = gobjects(0);
            obj.killFlashExpireTimes = [];
        end

        function generateTerrain(obj, type, varargin)
            xVec = 0:obj.resolution:obj.size.horiz;
            yVec = 0:obj.resolution:obj.size.vert;

            [X, Y] = meshgrid(xVec, yVec);
            Z = zeros(size(X));

            switch type
                case 'Flat'
                case 'Hills'
                    maxH = 20;
                    if ~isempty(varargin)
                        maxH = varargin{1};
                    end

                    centers = [obj.size.horiz * [0.2, 0.7, 0.5]; obj.size.vert * [0.3, 0.8, 0.4]];
                    widths = [15, 20, 10];
                    heights = [0.8 * maxH, 1.0 * maxH, 0.6 * maxH];

                    for i = 1:length(widths)
                        Z = Z + heights(i) * exp(-((X - centers(1, i)).^2 + (Y - centers(2, i)).^2) / (2 * widths(i)^2));
                    end
            end

            obj.terrain.X = X;
            obj.terrain.Y = Y;
            obj.terrain.Z = Z;

            obj.terrainProxy = griddedInterpolant({yVec, xVec}, Z, 'linear', 'nearest');

            obj.occMapCache = [];
            obj.occMapCacheThreshold = NaN;
        end

        function z = getElevation(obj, x, y)
            if isempty(obj.terrainProxy)
                z = zeros(size(x));
            else
                z = obj.terrainProxy(y, x);
            end
        end

        function occ = getOccMap(obj, elevThreshold)
            if nargin < 2 || isempty(elevThreshold)
                elevThreshold = 20;
            end

            if ~isempty(obj.occMapCache) && isequal(obj.occMapCacheThreshold, elevThreshold)
                occ = obj.occMapCache;
                return
            end

            if isempty(obj.terrain) || ~isfield(obj.terrain, 'Z') || isempty(obj.terrain.Z)
                occ = binaryOccupancyMap(zeros(obj.size.vert + 1, obj.size.horiz + 1));
                obj.occMapCache = occ;
                obj.occMapCacheThreshold = elevThreshold;
                return
            end

            Z = obj.terrain.Z;

            if obj.resolution ~= 1
                xq = 0:1:obj.size.horiz;
                yq = 0:1:obj.size.vert;
                [Xq, Yq] = meshgrid(xq, yq);
                Z = obj.getElevation(Xq, Yq);
            end

            costmap = (Z > elevThreshold);
            occ = binaryOccupancyMap(fliplr(costmap));

            obj.occMapCache = occ;
            obj.occMapCacheThreshold = elevThreshold;
        end

        function displayMap2D(obj, ax)
            if nargin >= 2 && ~isempty(ax) && isgraphics(ax, 'axes')
                axes(ax); %#ok<LAXES>
            end

            hold on
            axis equal
            grid on
            box on
            xlim([0, obj.size.horiz]);
            ylim([0, obj.size.vert]);
            title("UAS Simulation");
            xlabel("X (m)");
            ylabel("Y (m)");
        end

        function startAnimation(obj, AOR, assets, effectors, sensors, numUAS, hideClock, varargin)
            p = inputParser;
            p.addParameter('NFZ', [], @(x) isempty(x) || isa(x, 'polyshape'));
            p.addParameter('Obstacles', polyshape.empty, @(x) isempty(x) || isa(x, 'polyshape'));
            p.addParameter('Use2DRings', true, @(x) islogical(x) && isscalar(x));
            p.addParameter('Defenders', [], @(x) true);
            p.addParameter('Grid2D', true, @(x) islogical(x) && isscalar(x));
            p.parse(varargin{:});

            NFZ = p.Results.NFZ;
            obstacles = p.Results.Obstacles;
            use2DRings = p.Results.Use2DRings;
            defenders = p.Results.Defenders;
            grid2D = p.Results.Grid2D;

            obj.is3DView = ~grid2D;

            obj.killFlashHandles = gobjects(0);
            obj.killFlashExpireTimes = [];

            if grid2D
                if isempty(obj.figSim) || ~isgraphics(obj.figSim)
                    obj.figSim = figure('Name', 'UAS Simulation', 'NumberTitle', 'off');
                else
                    figure(obj.figSim);
                end

                clf(obj.figSim);
                obj.axSim = axes('Parent', obj.figSim);
                obj.displayMap2D(obj.axSim);

                if isempty(obj.figTerr) || ~isgraphics(obj.figTerr)
                    obj.figTerr = figure('Name', 'Terrain Elevation', 'NumberTitle', 'off');
                else
                    figure(obj.figTerr);
                end

                clf(obj.figTerr);
                obj.axTerr = axes('Parent', obj.figTerr);
                hold(obj.axTerr, 'on');

                if ~isempty(obj.terrain) && isfield(obj.terrain, 'Z') && ~isempty(obj.terrain.Z)
                    Z = obj.terrain.Z;

                    if obj.resolution ~= 1
                        xq = 0:1:obj.size.horiz;
                        yq = 0:1:obj.size.vert;
                        [Xq, Yq] = meshgrid(xq, yq); %#ok<ASGLU>
                        Z = obj.getElevation(Xq, Yq);
                        imagesc(obj.axTerr, xq, yq, Z);
                    else
                        xq = 0:obj.resolution:obj.size.horiz;
                        yq = 0:obj.resolution:obj.size.vert;
                        imagesc(obj.axTerr, xq, yq, Z);
                    end

                    set(obj.axTerr, 'YDir', 'normal');
                    axis(obj.axTerr, 'equal');
                    xlim(obj.axTerr, [0, obj.size.horiz]);
                    ylim(obj.axTerr, [0, obj.size.vert]);
                    box(obj.axTerr, 'on');
                    title(obj.axTerr, 'Terrain Elevation');
                    xlabel(obj.axTerr, 'X (m)');
                    ylabel(obj.axTerr, 'Y (m)');
                    colorbar(obj.axTerr);
                else
                    axis(obj.axTerr, 'equal');
                    xlim(obj.axTerr, [0, obj.size.horiz]);
                    ylim(obj.axTerr, [0, obj.size.vert]);
                    box(obj.axTerr, 'on');
                    title(obj.axTerr, 'Terrain Elevation');
                    xlabel(obj.axTerr, 'X (m)');
                    ylabel(obj.axTerr, 'Y (m)');
                end

                axes(obj.axSim); %#ok<LAXES>
            else
                if isempty(obj.figSim) || ~isgraphics(obj.figSim)
                    obj.figSim = figure('Name', 'UAS Simulation', 'NumberTitle', 'off');
                else
                    figure(obj.figSim);
                end

                clf(obj.figSim);
                obj.axSim = axes('Parent', obj.figSim);
                hold(obj.axSim, 'on');
                grid(obj.axSim, 'on');
                axis(obj.axSim, 'equal');
                xlim(obj.axSim, [0, obj.size.horiz]);
                ylim(obj.axSim, [0, obj.size.vert]);
                view(obj.axSim, 3);
                title(obj.axSim, "UAS Simulation (3D Terrain & Coverage)");
                xlabel(obj.axSim, "X (m)");
                ylabel(obj.axSim, "Y (m)");
                zlabel(obj.axSim, "Elevation (m)");
            end

            obj.UASTrail = gobjects(1, numUAS);
            obj.UASHead = gobjects(1, numUAS);

            for i = 1:numUAS
                if obj.is3DView
                    obj.UASTrail(i) = plot3(obj.axSim, NaN, NaN, NaN, 'r-', 'DisplayName', "Adversary Trail");
                    obj.UASHead(i) = plot3(obj.axSim, NaN, NaN, NaN, 'r^', 'LineStyle', 'none', 'DisplayName', "Adversary");
                else
                    obj.UASTrail(i) = plot(obj.axSim, NaN, NaN, 'r-', 'DisplayName', "Adversary Trail");
                    obj.UASHead(i) = plot(obj.axSim, NaN, NaN, 'r^', 'LineStyle', 'none', 'DisplayName', "Adversary");
                end

                if i > 1
                    set(obj.UASTrail(i), 'HandleVisibility', 'off');
                    set(obj.UASHead(i), 'HandleVisibility', 'off');
                end
            end

            if obj.is3DView
                obj.UASsensed = plot3(obj.axSim, NaN, NaN, NaN, 'ko', 'LineStyle', 'none', 'DisplayName', "Detection Ping");
                obj.UASkilled = plot3(obj.axSim, NaN, NaN, NaN, 'gx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', "Intercept");
                obj.UAScrashed = plot3(obj.axSim, NaN, NaN, NaN, 'kx', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', "Terrain Crash");
                obj.assetDestroyed = plot3(obj.axSim, NaN, NaN, NaN, 'rx', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Hit");
            else
                obj.UASsensed = plot(obj.axSim, NaN, NaN, 'ko', 'LineStyle', 'none', 'DisplayName', "Detection Ping");
                obj.UASkilled = plot(obj.axSim, NaN, NaN, 'gx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', "Intercept");
                obj.UAScrashed = plot(obj.axSim, NaN, NaN, 'kx', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', "Terrain Crash");
                obj.assetDestroyed = plot(obj.axSim, NaN, NaN, 'rx', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Hit");
            end

            if obj.is3DView
                aorVerts = AOR.Vertices;
                if ~isempty(aorVerts)
                    zA = obj.getElevation(aorVerts(:, 1), aorVerts(:, 2)) + 0.05;
                    plot3(obj.axSim, [aorVerts(:, 1); aorVerts(1, 1)], [aorVerts(:, 2); aorVerts(1, 2)], [zA; zA(1)], ...
                        'k-', 'LineWidth', 2, 'DisplayName', "AOR");
                end
            else
                plot(AOR, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2, 'DisplayName', "AOR");
            end

            if ~isempty(NFZ)
                if obj.is3DView
                    v = NFZ.Vertices;
                    if ~isempty(v)
                        zN = obj.getElevation(v(:, 1), v(:, 2)) + 0.1;
                        obj.NFZhandle = plot3(obj.axSim, [v(:, 1); v(1, 1)], [v(:, 2); v(1, 2)], [zN; zN(1)], ...
                            '--', 'Color', [1 1 0], 'LineWidth', 2, 'DisplayName', "NFZ");
                    else
                        obj.NFZhandle = gobjects(0);
                    end
                else
                    obj.NFZhandle = plot(NFZ, 'FaceColor', [1 1 0], 'FaceAlpha', 0.25, 'EdgeColor', [1 1 0], 'LineWidth', 2, 'DisplayName', "NFZ");
                end
            else
                obj.NFZhandle = gobjects(0);
            end

            if isempty(obstacles)
                obstacles = polyshape.empty;
            end

            obj.obstacleHandles = gobjects(length(obstacles), 1);
            for k = 1:length(obstacles)
                if obj.is3DView
                    v = obstacles(k).Vertices;
                    if isempty(v)
                        obj.obstacleHandles(k) = gobjects(1);
                    else
                        zO = obj.getElevation(v(:, 1), v(:, 2)) + 0.1;
                        obj.obstacleHandles(k) = plot3(obj.axSim, [v(:, 1); v(1, 1)], [v(:, 2); v(1, 2)], [zO; zO(1)], ...
                            '-', 'Color', [0 0 1], 'LineWidth', 2, 'DisplayName', "Obstacle");
                    end
                else
                    obj.obstacleHandles(k) = plot(obstacles(k), 'FaceColor', 'k', 'FaceAlpha', 1.0, 'EdgeColor', [0 0 1], 'LineWidth', 2, 'DisplayName', "Obstacle");
                end

                if k > 1 && isgraphics(obj.obstacleHandles(k))
                    set(obj.obstacleHandles(k), 'HandleVisibility', 'off');
                end
            end

            if ~hideClock
                if obj.is3DView
                    obj.timeBox = text(obj.axSim, 0.03 * obj.size.horiz, 0.95 * obj.size.vert, max(obj.terrain.Z(:)) + 3, 't: 0s', ...
                        'EdgeColor', 'k', 'BackgroundColor', 'w');
                else
                    obj.timeBox = text(obj.axSim, 0.03 * obj.size.horiz, 0.95 * obj.size.vert, 't: 0s', ...
                        'EdgeColor', 'k', 'BackgroundColor', 'w');
                end
            else
                obj.timeBox = [];
            end

            if obj.is3DView
                obj.eventBox = text(obj.axSim, 0.03 * obj.size.horiz, 0.90 * obj.size.vert, max(obj.terrain.Z(:)) + 1.5, '', ...
                    'EdgeColor', 'k', 'BackgroundColor', 'w', 'Color', 'k');
            else
                obj.eventBox = text(obj.axSim, 0.03 * obj.size.horiz, 0.90 * obj.size.vert, '', ...
                    'EdgeColor', 'k', 'BackgroundColor', 'w', 'Color', 'k');
            end

            obj.assets = assets;
            for i = 1:length(assets)
                x = assets(i).location(1);
                y = assets(i).location(2);

                if obj.is3DView
                    z = obj.getElevation(x, y) + map.assetHeight_();
                    hA = plot3(obj.axSim, x, y, z, 's', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', "Asset");
                else
                    hA = plot(obj.axSim, x, y, 's', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', "Asset");
                end

                if i > 1
                    set(hA, 'HandleVisibility', 'off');
                end
            end

            nEff = length(effectors);
            obj.effectorHandles = gobjects(nEff, 1);
            obj.effectorRingHandles = gobjects(nEff, 1);

            for i = 1:nEff
                loc = effectors(i).location;
                range = effectors(i).range;

                if use2DRings
                    hRing = obj.circleLine(loc, range, '--', 'c', 2, 0.05);
                    set(hRing, 'DisplayName', "Effector Radius");

                    if i > 1
                        set(hRing, 'HandleVisibility', 'off');
                    end

                    obj.effectorRingHandles(i) = hRing;
                else
                    obj.effectorRingHandles(i) = gobjects(1);
                end

                if obj.is3DView
                    z = obj.getElevation(loc(1), loc(2)) + map.effectorHeight_();
                    hE = plot3(obj.axSim, loc(1), loc(2), z, '.', 'Color', 'c', 'MarkerSize', 22, 'DisplayName', "Effector");
                else
                    hE = plot(obj.axSim, loc(1), loc(2), '.', 'Color', 'c', 'MarkerSize', 22, 'DisplayName', "Effector");
                end

                if i > 1
                    set(hE, 'HandleVisibility', 'off');
                end

                obj.effectorHandles(i) = hE;
            end

            nSens = length(sensors);
            obj.sensorHandles = gobjects(nSens, 1);
            obj.sensorRingHandles = gobjects(nSens, 1);

            for i = 1:nSens
                loc = sensors(i).location;
                range = sensors(i).range;

                if use2DRings
                    hRing = obj.circleLine(loc, range, '--', 'm', 2, 0.05);
                    set(hRing, 'DisplayName', "Sensor Radius");

                    if i > 1
                        set(hRing, 'HandleVisibility', 'off');
                    end

                    obj.sensorRingHandles(i) = hRing;
                else
                    obj.sensorRingHandles(i) = gobjects(1);
                end

                if obj.is3DView
                    z = obj.getElevation(loc(1), loc(2)) + map.sensorHeight_();
                    hS = plot3(obj.axSim, loc(1), loc(2), z, '.', 'Color', 'm', 'MarkerSize', 22, 'DisplayName', "Sensor");
                else
                    hS = plot(obj.axSim, loc(1), loc(2), '.', 'Color', 'm', 'MarkerSize', 22, 'DisplayName', "Sensor");
                end

                if i > 1
                    set(hS, 'HandleVisibility', 'off');
                end

                obj.sensorHandles(i) = hS;
            end

            if ~isempty(defenders)
                nDef = numel(defenders);
                obj.defenderHandles = gobjects(nDef, 1);
                obj.defenderTrailHandles = gobjects(nDef, 1);
                obj.defenderTrailHist = cell(nDef, 1);

                for d = 1:nDef
                    loc = map.getDefenderLoc_(defenders(d));

                    if obj.is3DView
                        hT = plot3(obj.axSim, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', "Defender Trail");
                    else
                        hT = plot(obj.axSim, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', "Defender Trail");
                    end
                    if d > 1
                        set(hT, 'HandleVisibility', 'off');
                    end
                    obj.defenderTrailHandles(d) = hT;
                    obj.defenderTrailHist{d} = loc;

                    if obj.is3DView
                        z = obj.getElevation(loc(1), loc(2)) + map.defenderHeight_();
                        hD = plot3(obj.axSim, loc(1), loc(2), z, 'o', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'b', 'MarkerFaceColor', 'b', 'DisplayName', "Defender");
                    else
                        hD = plot(obj.axSim, loc(1), loc(2), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'b', 'MarkerFaceColor', 'b', 'DisplayName', "Defender");
                    end
                    if d > 1
                        set(hD, 'HandleVisibility', 'off');
                    end
                    obj.defenderHandles(d) = hD;
                end
            else
                obj.defenderHandles = gobjects(0);
                obj.defenderTrailHandles = gobjects(0);
                obj.defenderTrailHist = {};
            end

            obj.projectileHandles = gobjects(0);

            if ~obj.is3DView
                view(0, 90);
            end
            xlim([0, obj.size.horiz]);
            ylim([0, obj.size.vert]);

            try
                legend off;
                lg = legend;
                if ~isempty(lg) && isvalid(lg)
                    delete(lg);
                end
            catch
            end
        end

        function updateDefenders(obj, defenders)
            obj.cleanupKillFlashes_();

            if isempty(defenders)
                return
            end

            if ~isempty(obj.defenderHandles)
                n = min(numel(defenders), numel(obj.defenderHandles));
                for d = 1:n
                    if ~isgraphics(obj.defenderHandles(d))
                        continue
                    end
                    loc = map.getDefenderLoc_(defenders(d));
                    if obj.is3DView
                        z = obj.getElevation(loc(1), loc(2)) + map.defenderHeight_();
                        set(obj.defenderHandles(d), 'XData', loc(1), 'YData', loc(2), 'ZData', z);
                    else
                        set(obj.defenderHandles(d), 'XData', loc(1), 'YData', loc(2));
                    end
                end
            end

            if ~isempty(obj.defenderTrailHandles) && ~isempty(obj.defenderTrailHist)
                n = min(numel(defenders), numel(obj.defenderTrailHandles));
                for d = 1:n
                    if ~isgraphics(obj.defenderTrailHandles(d))
                        continue
                    end

                    loc = map.getDefenderLoc_(defenders(d));
                    hist = obj.defenderTrailHist{d};

                    if isempty(hist)
                        hist = loc;
                    else
                        hist = [hist; loc]; %#ok<AGROW>
                    end

                    if size(hist, 1) > obj.defenderMaxTrailPoints
                        hist = hist(end - obj.defenderMaxTrailPoints + 1:end, :);
                    end

                    obj.defenderTrailHist{d} = hist;

                    if obj.is3DView
                        zHist = obj.getElevation(hist(:, 1), hist(:, 2)) + map.defenderTrailHeight_();
                        set(obj.defenderTrailHandles(d), 'XData', hist(:, 1), 'YData', hist(:, 2), 'ZData', zHist);
                    else
                        set(obj.defenderTrailHandles(d), 'XData', hist(:, 1), 'YData', hist(:, 2));
                    end
                end
            end
        end

        function updateEffectors(obj, effectors)
            obj.cleanupKillFlashes_();

            if isempty(obj.effectorHandles) || isempty(effectors)
                return
            end

            n = min(numel(effectors), numel(obj.effectorHandles));
            for i = 1:n
                if ~isgraphics(obj.effectorHandles(i))
                    continue
                end

                loc = effectors(i).location;

                if obj.is3DView
                    z = obj.getElevation(loc(1), loc(2)) + map.effectorHeight_();
                    set(obj.effectorHandles(i), 'XData', loc(1), 'YData', loc(2), 'ZData', z);
                else
                    set(obj.effectorHandles(i), 'XData', loc(1), 'YData', loc(2));
                end

                if i <= numel(obj.effectorRingHandles) && isgraphics(obj.effectorRingHandles(i))
                    obj.updateCircle_(obj.effectorRingHandles(i), loc, effectors(i).range, 0.05);
                end
            end
        end

        function updateProjectiles(obj, projectilePositions)
            obj.cleanupKillFlashes_();

            if nargin < 2 || isempty(projectilePositions)
                for k = 1:numel(obj.projectileHandles)
                    if isgraphics(obj.projectileHandles(k))
                        if obj.is3DView
                            set(obj.projectileHandles(k), 'XData', nan, 'YData', nan, 'ZData', nan, 'Visible', 'off');
                        else
                            set(obj.projectileHandles(k), 'XData', nan, 'YData', nan, 'Visible', 'off');
                        end
                    end
                end
                return
            end

            nProj = size(projectilePositions, 1);

            if isempty(obj.projectileHandles)
                obj.projectileHandles = gobjects(0);
            end

            if numel(obj.projectileHandles) < nProj
                for k = numel(obj.projectileHandles) + 1:nProj
                    if obj.is3DView
                        obj.projectileHandles(k, 1) = plot3(obj.axSim, nan, nan, nan, 'k.', 'MarkerSize', 16, 'DisplayName', 'Projectile');
                    else
                        obj.projectileHandles(k, 1) = plot(obj.axSim, nan, nan, 'k.', 'MarkerSize', 16, 'DisplayName', 'Projectile');
                    end
                    if k > 1
                        set(obj.projectileHandles(k), 'HandleVisibility', 'off');
                    end
                end
            end

            for k = 1:nProj
                if isgraphics(obj.projectileHandles(k))
                    if obj.is3DView && size(projectilePositions, 2) >= 3
                        set(obj.projectileHandles(k), ...
                            'XData', projectilePositions(k, 1), ...
                            'YData', projectilePositions(k, 2), ...
                            'ZData', projectilePositions(k, 3), ...
                            'Visible', 'on');
                    else
                        set(obj.projectileHandles(k), ...
                            'XData', projectilePositions(k, 1), ...
                            'YData', projectilePositions(k, 2), ...
                            'Visible', 'on');
                    end
                end
            end

            for k = nProj + 1:numel(obj.projectileHandles)
                if isgraphics(obj.projectileHandles(k))
                    if obj.is3DView
                        set(obj.projectileHandles(k), 'XData', nan, 'YData', nan, 'ZData', nan, 'Visible', 'off');
                    else
                        set(obj.projectileHandles(k), 'XData', nan, 'YData', nan, 'Visible', 'off');
                    end
                end
            end
        end

        function setEventMessage(obj, msg)
            obj.cleanupKillFlashes_();

            if ~isempty(obj.eventBox) && isvalid(obj.eventBox)
                set(obj.eventBox, 'String', msg);
            end
        end

        function animateUAScrashed(obj, position)
            obj.cleanupKillFlashes_();

            if obj.is3DView
                set(obj.UAScrashed, 'XData', position(1), 'YData', position(2), 'ZData', position(3));
            else
                set(obj.UAScrashed, 'XData', position(1), 'YData', position(2));
            end
        end

        function updateUASAnimation(obj, UASPos_all)
            obj.cleanupKillFlashes_();

            for i = 1:length(obj.UASTrail)
                if i <= length(UASPos_all)
                    pos = UASPos_all{i};
                    if ~isempty(pos)
                        if size(pos, 1) > obj.maxTrailPoints
                            pos = pos(end - obj.maxTrailPoints + 1:end, :);
                        end

                        if obj.is3DView
                            set(obj.UASTrail(i), 'XData', pos(:, 1), 'YData', pos(:, 2), 'ZData', pos(:, 3));
                            set(obj.UASHead(i), 'XData', pos(end, 1), 'YData', pos(end, 2), 'ZData', pos(end, 3));
                        else
                            set(obj.UASTrail(i), 'XData', pos(:, 1), 'YData', pos(:, 2));
                            set(obj.UASHead(i), 'XData', pos(end, 1), 'YData', pos(end, 2));
                        end
                    end
                end
            end
        end

        function animateDestroyedAssets(obj, assets, destroyedAssets)
            obj.cleanupKillFlashes_();

            XData = [];
            YData = [];
            ZData = [];

            for i = 1:length(destroyedAssets)
                x = assets(destroyedAssets(i)).location(1);
                y = assets(destroyedAssets(i)).location(2);
                XData(end + 1) = x; %#ok<AGROW>
                YData(end + 1) = y; %#ok<AGROW>
                if obj.is3DView
                    ZData(end + 1) = obj.getElevation(x, y) + map.assetHeight_(); %#ok<AGROW>
                end
            end

            if obj.is3DView
                set(obj.assetDestroyed, 'XData', XData, 'YData', YData, 'ZData', ZData);
            else
                set(obj.assetDestroyed, 'XData', XData, 'YData', YData);
            end
        end

        function animateUASsensed(obj, position)
            obj.cleanupKillFlashes_();

            if isempty(position)
                return
            end
            x = position(:, 2);
            y = position(:, 3);

            if obj.is3DView
                z = position(:, 4);
                set(obj.UASsensed, 'XData', x, 'YData', y, 'ZData', z);
            else
                set(obj.UASsensed, 'XData', x, 'YData', y);
            end
        end

        function animateUASkilled(obj, position)
            obj.cleanupKillFlashes_();

            if obj.is3DView
                set(obj.UASkilled, 'XData', position(1), 'YData', position(2), 'ZData', position(3));
            else
                set(obj.UASkilled, 'XData', position(1), 'YData', position(2));
            end

            obj.createKillFlash_(position);
        end

        function updateClock(obj, time)
            obj.cleanupKillFlashes_();

            if ~isempty(obj.timeBox) && isvalid(obj.timeBox)
                set(obj.timeBox, 'String', ['t: ', sprintf('%.2f', time), 's']);
            end
        end
    end

    methods (Access = private)
        function h = circleLine(obj, center, R, lineStyle, colorChar, lw, zOffset)
            if nargin < 7 || isempty(zOffset)
                zOffset = 0;
            end

            th = linspace(0, 2 * pi, 180);
            x = center(1) + R * cos(th);
            y = center(2) + R * sin(th);

            if obj.is3DView
                z = obj.getElevation(x, y) + zOffset;
                h = plot3(obj.axSim, x, y, z, 'LineStyle', lineStyle, 'LineWidth', lw, 'Color', colorChar);
            else
                h = plot(obj.axSim, x, y, 'LineStyle', lineStyle, 'LineWidth', lw, 'Color', colorChar);
            end
        end

        function updateCircle_(obj, h, center, R, zOffset)
            if nargin < 5 || isempty(zOffset)
                zOffset = 0;
            end

            th = linspace(0, 2 * pi, 180);
            x = center(1) + R * cos(th);
            y = center(2) + R * sin(th);

            if obj.is3DView
                z = obj.getElevation(x, y) + zOffset;
                set(h, 'XData', x, 'YData', y, 'ZData', z);
            else
                set(h, 'XData', x, 'YData', y);
            end
        end

        function createKillFlash_(obj, position)
            if isempty(obj.axSim) || ~isgraphics(obj.axSim, 'axes')
                return
            end

            holdState = ishold(obj.axSim);
            hold(obj.axSim, 'on');

            if obj.is3DView
                x = position(1);
                y = position(2);
                z = position(3);

                h1 = plot3(obj.axSim, x, y, z, 'yo', ...
                    'MarkerSize', 18, ...
                    'LineWidth', 2.5, ...
                    'HandleVisibility', 'off');

                h2 = plot3(obj.axSim, x, y, z, 'yx', ...
                    'MarkerSize', 20, ...
                    'LineWidth', 2.5, ...
                    'HandleVisibility', 'off');

                newHandles = [h1; h2];
            else
                x = position(1);
                y = position(2);

                h1 = plot(obj.axSim, x, y, 'yo', ...
                    'MarkerSize', 18, ...
                    'LineWidth', 2.5, ...
                    'HandleVisibility', 'off');

                h2 = plot(obj.axSim, x, y, 'yx', ...
                    'MarkerSize', 20, ...
                    'LineWidth', 2.5, ...
                    'HandleVisibility', 'off');

                newHandles = [h1; h2];
            end

            if ~holdState
                hold(obj.axSim, 'off');
            end

            expireTime = tic;
            for k = 1:numel(newHandles)
                obj.killFlashHandles(end + 1, 1) = newHandles(k); %#ok<AGROW>
                obj.killFlashExpireTimes(end + 1, 1) = toc(expireTime) + obj.killFlashDuration; %#ok<AGROW>
            end
        end

        function cleanupKillFlashes_(obj)
            if isempty(obj.killFlashHandles)
                return
            end

            keep = true(numel(obj.killFlashHandles), 1);

            for k = 1:numel(obj.killFlashHandles)
                h = obj.killFlashHandles(k);

                if ~isgraphics(h)
                    keep(k) = false;
                    continue
                end

                ud = getappdata(obj.axSim, 'KillFlashClockStart');
                if isempty(ud)
                    setappdata(obj.axSim, 'KillFlashClockStart', tic);
                    ud = getappdata(obj.axSim, 'KillFlashClockStart');
                end

                elapsed = toc(ud);
                if elapsed >= obj.killFlashExpireTimes(k)
                    delete(h);
                    keep(k) = false;
                end
            end

            obj.killFlashHandles = obj.killFlashHandles(keep);
            obj.killFlashExpireTimes = obj.killFlashExpireTimes(keep);
        end
    end

    methods (Static, Access = private)
        function loc = getDefenderLoc_(def)
            loc = def.location;
            loc = loc(:).';
            loc = loc(1:2);
        end

        function h = defenderHeight_()
            h = 0.8;
        end

        function h = defenderTrailHeight_()
            h = 0.2;
        end

        function h = effectorHeight_()
            h = 1.2;
        end

        function h = sensorHeight_()
            h = 1.0;
        end

        function h = assetHeight_()
            h = 0.8;
        end
    end
end
