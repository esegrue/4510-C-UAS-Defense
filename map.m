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

        function startAnimation(obj, AOR, assets, effectors, sensors, numUAS, hideClock)
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

            axesChildren = get(gca, 'Children');
            axesMatch = findobj(axesChildren, 'DisplayName', "AOR");       

            if isempty(axesMatch)
                plot(AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05, 'DisplayName', "AOR");

                if ~hideClock
                    obj.timeBox = text(0.05*obj.size.vert, 0.95*obj.size.vert, 50, 't: 0s', ...
                        'ColorMode', 'auto', 'EdgeColor', 'k', 'BackgroundColor', 'w');
                end

                for i = 1:length(assets)
                    x = assets(i).location(1); y = assets(i).location(2); z = obj.getElevation(x, y);
                    plot3(x, y, z+z_offset, 'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle','none' , 'DisplayName', "Asset " + i);
                end
                
                for i = 1:length(effectors)
                    loc = effectors(i).location; range = effectors(i).range;
                    obj.drawCoverageDome(loc, range, 'c');
                    z = obj.getElevation(loc(1), loc(2));
                    plot3(loc(1), loc(2), z+z_offset, '.', 'Color', 'c', 'DisplayName', "Effector " + i, 'MarkerSize', 20)
                end
                
                for i = 1:length(sensors)
                    loc = sensors(i).location; range = sensors(i).range;
                    obj.drawCoverageDome(loc, range, 'm');
                    z = obj.getElevation(loc(1), loc(2));
                    plot3(loc(1), loc(2), z+z_offset, '.', 'Color', 'm', 'DisplayName', "Sensor " + i, 'MarkerSize', 20)
                end
            end
            xlim([0,obj.size.horiz]); ylim([0,obj.size.vert]);
        end

        function animateUAScrashed(obj, position)
            set(obj.UAScrashed, 'XData', position(1), 'YData', position(2), 'ZData', position(3));
        end

        function drawCoverageDome(obj, pos, range, color)
            [th, phi] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi/2, 15));
            [x_sphere, y_sphere, z_sphere] = sph2cart(th, phi, range);
            x_sphere = x_sphere + pos(1); y_sphere = y_sphere + pos(2);
            z_center = obj.getElevation(pos(1), pos(2));
            z_sphere = z_sphere + z_center;
            surf(x_sphere, y_sphere, z_sphere, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'DisplayName', 'Coverage Dome');
        end

        function updateUASAnimation(obj, UASPos_all)
            for i = 1:length(obj.UASTrail)
                if i <= length(UASPos_all)
                    pos = UASPos_all{i}; 
                    if ~isempty(pos)
                        set(obj.UASTrail(i), 'XData', pos(:, 1), 'YData', pos(:, 2), 'ZData', pos(:, 3));
                        set(obj.UASHead(i), 'XData', pos(end, 1), 'YData', pos(end, 2), 'ZData', pos(end, 3));
                    end
                end
            end
        end

        function animateDestroyedAssets(obj, assets, destroyedAssets)
            XData = []; YData = []; ZData = [];
            for i = 1:length(destroyedAssets)
                x = assets(destroyedAssets(i)).location(1); y = assets(destroyedAssets(i)).location(2); z = obj.getElevation(x, y) + 1; 
                XData(end+1) = x; YData(end+1) = y; ZData(end+1) = z;
            end
            set(obj.assetDestroyed, 'XData', XData, 'YData', YData, 'ZData', ZData)
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
        end
    end
end