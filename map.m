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
        
        effectorDomes
        effectorDots
    end
    
    methods
        function obj = map(vertical, horizontal, resolution)
            if nargin < 3; resolution = 1; end
            obj.size.vert = vertical;
            obj.size.horiz = horizontal;
            obj.resolution = resolution;
            obj.generateTerrain('Flat');
        end

        function generateTerrain(obj, type, varargin)
            x_vec = 0:obj.resolution:obj.size.horiz;
            y_vec = 0:obj.resolution:obj.size.vert;
            
            [X, Y] = meshgrid(x_vec, y_vec);
            Z = zeros(size(X));
            
            switch type
                case 'Flat'
                case 'Hills'
                    maxH = 20;
                    if ~isempty(varargin); maxH = varargin{1}; end
                    centers = [obj.size.horiz * [0.2, 0.7, 0.5]; ...
                               obj.size.vert  * [0.3, 0.8, 0.4]];
                    widths = [15, 20, 10]; 
                    heights = [0.8*maxH, 1.0*maxH, 0.6*maxH];
                    for i = 1:length(widths)
                        Z = Z + heights(i) * exp(-((X-centers(1,i)).^2 + ...
                            (Y-centers(2,i)).^2) / (2*widths(i)^2));
                    end
            end
            obj.terrain.X = X; obj.terrain.Y = Y; obj.terrain.Z = Z;
            obj.terrainProxy = griddedInterpolant({y_vec, x_vec}, Z, 'linear', 'nearest');
        end

        function z = getElevation(obj, x, y)
            if isempty(obj.terrainProxy)
                z = zeros(size(x));
            else
                z = obj.terrainProxy(y, x);
            end
        end

        function displayMap(obj) 
            hold on
            if ~isempty(obj.terrain)
                surf(obj.terrain.X, obj.terrain.Y, obj.terrain.Z, ...
                    'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Terrain');
                colormap(summer); colorbar;
            end
            xlim([0,obj.size.horiz]); ylim([0,obj.size.vert]);
            grid on; axis equal; view(45, 30);
            title("UAS Simulation (3D Terrain & Coverage)");
            xlabel("X (m)"); ylabel("Y (m)"); zlabel("Elevation (m)");
        end

        function startAnimation(obj, asset, effectors, sensors, numUAS, hideClock, nfzs)
            if nargin < 7; nfzs = polyshape.empty; end

            if isempty(get(groot, 'CurrentFigure'))
                figure('Name', 'Simulation Animation', 'Position', [100 100 600 400]);
            end

            obj.displayMap
            obj.UASTrail = gobjects(1, numUAS);
            obj.UASHead = gobjects(1, numUAS);
            z_offset = 1; 

            for i = 1:numUAS
                obj.UASTrail(i) = plot3(NaN, NaN, NaN, 'Color', 'r', 'DisplayName', "UAS Trail " + i);
                obj.UASHead(i) = plot3(NaN, NaN, NaN, 'Color', 'r', 'Marker', '^', 'DisplayName', "UAS " + i);
            end

            obj.UASsensed = plot3(NaN, NaN, NaN, 'Color', 'k', 'Marker', 'o', 'LineStyle', 'none', 'DisplayName', "Detection");
            obj.UASkilled = plot3(NaN, NaN, NaN, 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12, 'DisplayName', "Kill Event");
            obj.UAScrashed = plot3(NaN, NaN, NaN, 'Marker', 'x', 'Color', 'k', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', "Terrain Crash");
            obj.assetDestroyed = plot3(NaN, NaN, NaN, 'Marker', 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Destroyed");

            if ~hideClock
                obj.timeBox = text(0.05*obj.size.vert, 0.95*obj.size.vert, 50, 't: 0s', ...
                    'ColorMode', 'auto', 'EdgeColor', 'k', 'BackgroundColor', 'w');
            end

            x = asset.location(1); y = asset.location(2); z = obj.getElevation(x, y);
            plot3(x, y, z+z_offset, 'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle','none' , 'DisplayName', "Asset");
            
            % Draw No-Fly Zones
            for n = 1:length(nfzs)
                [vx, vy] = boundary(nfzs(n));
                vz = zeros(size(vx));
                for v = 1:length(vx)
                    vz(v) = obj.getElevation(vx(v), vy(v)) + z_offset;
                end
                fill3(vx, vy, vz, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'No-Fly Zone');
            end

            % Initialize Effector graphics arrays
            obj.effectorDomes = gobjects(1, length(effectors));
            obj.effectorDots = gobjects(1, length(effectors));

            for i = 1:length(effectors)
                loc = effectors(i).location; range = effectors(i).range;
                color = 'c';
                if isfield(effectors(i), 'mode') && effectors(i).mode == "MOBILE"
                    color = 'b';
                end
                
                [th, phi] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi/2, 15));
                [x_sphere, y_sphere, z_sphere] = sph2cart(th, phi, range);
                x_sphere = x_sphere + loc(1); y_sphere = y_sphere + loc(2);
                z_center = obj.getElevation(loc(1), loc(2));
                z_sphere = z_sphere + z_center;
                
                obj.effectorDomes(i) = surf(x_sphere, y_sphere, z_sphere, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'DisplayName', "Effector Dome " + i);
                obj.effectorDots(i) = plot3(loc(1), loc(2), z_center+z_offset, '.', 'Color', color, 'DisplayName', "Effector " + i, 'MarkerSize', 20);
            end
            
            for i = 1:length(sensors)
                loc = sensors(i).location; range = sensors(i).range;
                [th, phi] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi/2, 15));
                [x_sphere, y_sphere, z_sphere] = sph2cart(th, phi, range);
                x_sphere = x_sphere + loc(1); y_sphere = y_sphere + loc(2);
                z_center = obj.getElevation(loc(1), loc(2));
                z_sphere = z_sphere + z_center;
                surf(x_sphere, y_sphere, z_sphere, 'FaceColor', 'm', 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'DisplayName', 'Sensor Coverage');
                plot3(loc(1), loc(2), z_center+z_offset, '.', 'Color', 'm', 'DisplayName', "Sensor " + i, 'MarkerSize', 20)
            end
            
            xlim([0,obj.size.horiz]); ylim([0,obj.size.vert]);
        end

        function updateEffectors(obj, effectors)
            z_offset = 1;
            for i = 1:length(effectors)
                if isfield(effectors(i), 'mode') && effectors(i).mode == "MOBILE"
                    loc = effectors(i).location;
                    range = effectors(i).range;
                    z_center = obj.getElevation(loc(1), loc(2));
                    
                    set(obj.effectorDots(i), 'XData', loc(1), 'YData', loc(2), 'ZData', z_center + z_offset);
                    
                    [th, phi] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi/2, 15));
                    [x_sphere, y_sphere, z_sphere] = sph2cart(th, phi, range);
                    x_sphere = x_sphere + loc(1); 
                    y_sphere = y_sphere + loc(2);
                    z_sphere = z_sphere + z_center;
                    
                    set(obj.effectorDomes(i), 'XData', x_sphere, 'YData', y_sphere, 'ZData', z_sphere);
                end
            end
        end

        function animateUAScrashed(obj, position)
            set(obj.UAScrashed, 'XData', position(1), 'YData', position(2), 'ZData', position(3));
        end

        function updateUASAnimation(obj, UASPos_all)
            for i = 1:length(obj.UASTrail)
                if i <= length(UASPos_all)
                    if ~isvalid(obj.UASTrail(i)); continue; end
                    pos = UASPos_all{i}; 
                    if ~isempty(pos)
                        set(obj.UASTrail(i), 'XData', pos(:, 1), 'YData', pos(:, 2), 'ZData', pos(:, 3));
                        set(obj.UASHead(i), 'XData', pos(end, 1), 'YData', pos(end, 2), 'ZData', pos(end, 3));
                    end
                end
            end
        end

        function animateDestroyedAsset(obj, asset)
            x = asset.location(1); y = asset.location(2); z = obj.getElevation(x, y) + 1; 
            set(obj.assetDestroyed, 'XData', x, 'YData', y, 'ZData', z)
        end

        function animateUASsensed(obj, position)
            if isempty(position); return; end
            x = position(:, 2); y = position(:, 3); z = position(:, 4);
            set(obj.UASsensed, 'XData', x, 'YData', y, 'ZData', z)
        end

        function animateUASkilled(obj, position)
             set(obj.UASkilled, 'XData', position(1), 'YData', position(2), 'ZData', position(3))
        end

        function updateClock(obj, time)
            if ~isempty(obj.timeBox) && isvalid(obj.timeBox)
                set(obj.timeBox, 'String', ['t: ', sprintf('%.2f', time), 's']);
            end
        end

        function wipeAnimation(obj)
            clf; obj.timeBox = [];
            obj.UASTrail = gobjects(0);
            obj.UASHead = gobjects(0);
            obj.UASsensed = gobjects(0);
            obj.UASkilled = gobjects(0);
            obj.UAScrashed = gobjects(0);
            obj.assetDestroyed = gobjects(0);
            obj.effectorDomes = gobjects(0);
            obj.effectorDots = gobjects(0);
        end
    end
end