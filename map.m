classdef map < handle
    properties
        size
        UASTrail
        UASHead
        UASsensed
        UASkilled
        assetDestroyed
        assets
        NFZs
        timeBox
    end
    methods
        function obj = map(vertical, horizontal)
            obj.size.vert = vertical;
            obj.size.horiz = horizontal;
        end

        function displayMap(obj) 
            hold on
            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])
            grid on
            axis equal
            title("UAS Simulation")
            xlabel("X (m)")
            ylabel("Y (m)")
        end

        function startAnimation(obj, AOR, assets, NFZs, effectors, sensors, numUAS, hideClock)
            obj.displayMap
            
            obj.UASTrail = gobjects(1, numUAS);
            obj.UASHead = gobjects(1, numUAS);
            
            for i = 1:numUAS
                obj.UASTrail(i) = plot(NaN, NaN, 'Color', 'r', 'DisplayName', "UAS Trail " + i);
                obj.UASHead(i) = plot(NaN, NaN, 'Color', 'r', 'Marker', '^', 'DisplayName', "UAS " + i);
            end

            obj.UASsensed = plot(NaN, NaN, 'Color', 'k', 'Marker', 'o', 'LineStyle', 'none', 'DisplayName', "Detection");
            obj.UASkilled = plot(NaN, NaN, 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12, 'DisplayName', "Kill Event");
            obj.assetDestroyed = plot(NaN, NaN, 'Marker', 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Destroyed");

            axesChildren = get(gca, 'Children');
            axesMatch = findobj(axesChildren, 'DisplayName', "AOR");       

            if isempty(axesMatch)
                plot(AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05, 'DisplayName', "AOR");

                if ~hideClock
                    obj.timeBox = text(0.05*obj.size.vert, 0.95*obj.size.vert, 't: 0s', 'ColorMode', 'auto', 'EdgeColor', 'k');
                end

                for i = 1:length(assets)
                    plot(assets(i).location(1), assets(i).location(2), 'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle','none' , 'DisplayName', "Asset " + i);
                end

                if ~isempty(NFZs)
                    for i = 1:length(NFZs)
                        plot(NFZs(i), 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'y', 'DisplayName', "NFZ " + i);
                    end
                end
                
                for i = 1:length(effectors)
                    x = effectors(i).location(1);
                    y = effectors(i).location(2);
                    r = effectors(i).range;
                    rectangle('Position',[x-r, y-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', 'c', 'EdgeColor', 'c', 'LineStyle', '--', 'FaceAlpha', 0.05);
                    plot(x, y, '.', 'Color', 'c', 'DisplayName', "Effector " + i, 'MarkerSize', 20)
                end
                
                for i = 1:length(sensors)
                    x = sensors(i).location(1);
                    y = sensors(i).location(2);
                    r = sensors(i).range;
                    rectangle('Position',[x-r, y-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', 'm', 'EdgeColor', 'm', 'LineStyle', '--', 'FaceAlpha', 0.05);
                    plot(x, y, '.', 'Color', 'm', 'DisplayName', "Sensor " + i, 'MarkerSize', 20)
                end
            end
            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])
        end

        function updateUASAnimation(obj, UASPos_all)
            for i = 1:length(obj.UASTrail)
                if i <= length(UASPos_all)
                    pos = UASPos_all{i};
                    if ~isempty(pos)
                        set(obj.UASTrail(i), 'XData', pos(:, 1), 'YData', pos(:, 2));
                        set(obj.UASHead(i), 'XData', pos(end, 1), 'YData', pos(end, 2));
                    end
                end
            end
        end

        function animateDestroyedAssets(obj, assets, destroyedAssets)
            XData = [];
            YData = [];
            for i = 1:length(destroyedAssets)
                XData(1, i) = assets(destroyedAssets(i)).location(1);
                YData(1, i) = assets(destroyedAssets(i)).location(2);
            end
            set(obj.assetDestroyed, 'XData', XData, 'YData', YData)
        end

        function animateUASsensed(obj, position)
            set(obj.UASsensed, 'XData', position(:,2), 'YData', position(:,3))
        end

        function animateUASkilled(obj, position)
            set(obj.UASkilled, 'XData', position(1), 'YData', position(2))
        end

        function updateClock(obj, time)
            set(obj.timeBox, 'String', ['t: ', sprintf('%.2f', time), 's']);
        end

        function wipeAnimation(obj)
            clf
            obj.timeBox = [];
        end
    end
end