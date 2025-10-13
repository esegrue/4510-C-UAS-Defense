classdef map < handle
    properties
        size
        UASTrail
        UASHead
        UASSensed
        UASDestroyed
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

        function displayMap(obj) % Display the initial map size and labels
            hold on
            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])
            grid on
            axis equal
            title("UAS Simulation")
            xlabel("X (m)")
            ylabel("Y (m)")
        end

        % Initialize animation
        function startAnimation(obj, AOR, assets, NFZs, sensors, hideClock)
            obj.displayMap
            
            obj.UASTrail = plot(NaN, NaN, 'Color', 'r', 'DisplayName', "UAS Trail");
            obj.UASHead = plot(NaN, NaN, 'Color', 'r', 'Marker', '^', 'DisplayName', "UAS");
            obj.UASSensed = plot(NaN, NaN, 'Color', 'y', 'Marker', 'square', 'LineStyle', 'none', 'DisplayName', "UAS Sensor Detection Point");
            obj.UASDestroyed = plot(NaN, NaN, 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12);
            obj.assetDestroyed = plot(NaN, NaN, 'Marker', 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Destroyed");

            % Determine if plot has already been initialized
            axesChildren = get(gca, 'Children');
            axesMatch = findobj(axesChildren, 'DisplayName', "AOR");       % I, Daniel Burns, do recognize that this is quite possibly the worst way to make this check.

            if isempty(axesMatch)
                % Plot AOR
                plot(AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05, 'DisplayName', "AOR");

                if hideClock == false
                    obj.timeBox = text(0.05*obj.size.vert, 0.95*obj.size.vert, 't: 0s', 'ColorMode', 'auto', 'EdgeColor', 'k');
                end

                % Plot assets
                for i = 1:length(assets)
                    obj.assets = plot(assets(i).location(1), assets(i).location(2), 'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle','none' , 'DisplayName', "Asset " + i);
                end

                % Plot NFZs
                if isempty(NFZs) == 0
                    for i = 1:length(NFZs)
                        obj.NFZs = plot(NFZs(i), 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'y', 'DisplayName', "NFZ " + i);
                    end
                end

                % Plot sensors
                for i = 1:length(sensors)
                    x = sensors(i).location(1);
                    y = sensors(i).location(2);
                    r = sensors(i).range;

                    rectangle('Position',[x-r, y-r, 2*r, 2*r], ...
                        'Curvature', [1 1], ...
                        'FaceColor', 'c', ...
                        'EdgeColor', 'c', ...
                        'LineStyle', '--', ...
                        'FaceAlpha', 0.05)
                    plot(x, y, '.', 'Color', 'c', 'DisplayName', "Sensor " + i, 'MarkerSize', 20)
                end
            end

            

            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])
        end
        function updateUASAnimation(obj, UASPos)
            set(obj.UASTrail, 'XData', UASPos(:, 1), 'YData', UASPos(:, 2))
            set(obj.UASHead, 'XData', UASPos(end, 1), 'YData', UASPos(end, 2))
        end

        function updateSensedLocations(obj, sensedPos)
            set(obj.UASSensed, 'XData', sensedPos(:, 1), 'YData', sensedPos(:, 2))
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

        function animateUASDestroyed(obj, position)
            set(obj.UASDestroyed, 'XData', position(1), 'YData', position(2))
        end

        function updateClock(obj, time)
            set(obj.timeBox, 'String', ['t: ', sprintf('%.2f', time), 's']);
        end

        function cleanAnimation(obj)
            obj.timeBox = [];
        end

        function wipeAnimation(obj)
            clf
            obj.timeBox = [];
        end
    end
end