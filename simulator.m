classdef simulator
    %SIMULATOR for C-UAS Sensor Placement
    %   This is the main simulator class for the UAS-Sensor Placement. This
    %   is where time will march, and from here all map updates are called.

    properties
        map
        AOR
        UAS
        sensors
        assets
        tick
        dt
        tps
        animate
        NFZs
        resetGraphics
        animationMultiplier
        hideClock
    end

    methods
        function obj = simulator(map, aor, uas, sensors, assets, options)
            %SIMULATOR
            arguments
                map
                aor
                uas
                sensors
                assets
                options.tps double = 20                                    % How many ticks per second the logical system should operate in, AKA: simulation resolution
                options.animate logical = true                             % Set false if you just want data
                options.nfzs polyshape = polyshape.empty                   % This is where you declare any NFZs you want, should you decide to do so
                options.resetGraphics logical = true                       % This will reset all graphics if used over multiple iterations FOR THE SAME MAP! As such, default is true
                options.animationMultiplier double = 1                     % Animation speed multiplier, default 1x
                options.hideClock logical = false                          % Set true if you want to hide the clock
            end
            obj.map = map;
            obj.AOR = aor;
            obj.UAS = uas;
            obj.sensors = sensors;
            obj.assets = assets;
            obj.tick = 0;
            obj.tps = options.tps;
            obj.dt = 1 / obj.tps;
            obj.animate = options.animate;
            obj.NFZs = options.nfzs;
            obj.resetGraphics = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier;
            obj.hideClock = options.hideClock;
        end

        function results = runSim(obj)
            destroyedAssets = [];                                          % Initialize assets destroyed
            UASSensed = 0;                                                 % Initialize UAS sensed count
            NFZEntered = false;                                            % Initialize NFZ entry status
            lastTick = false;                                              % Initialize lastTick to be set true when simulation should end
            UASSensedPos = [];                                             % Initialize matrix to track all positions in which the UAS is sensed
            UASPos = [obj.UAS.position(1), obj.UAS.position(2)];           % This matrix tracks all current and previous UAS positions
            
            if obj.animate == true
                if obj.resetGraphics
                    obj.map.wipeAnimation()
                end
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.sensors, obj.hideClock);
            end
<<<<<<< Updated upstream

=======
            
>>>>>>> Stashed changes
            % Generate Sensor Contours
            for i = 1:length(obj.sensors)
                
            end

            while lastTick == false
                if obj.tick ~= 0
                    % Determine new UAS Position
                    if obj.UAS.mode == "Linear"
                        obj.UAS.linearMotion(obj.dt);
                    elseif obj.UAS.mode == "Search"
                        obj.UAS.searchMotion(obj.dt,obj.assets, destroyedAssets);
                    else
                        error("Improperly defined UAS mode. Simulation terminating.") % Move this check and error to UAS initialization???
                    end

                    % Update local UASPos
                    UASPos = cat(1, UASPos, obj.UAS.position);
                end

                % Check for any logical events
                [eventSensor] = obj.checkSensorCollision(UASPos(end, :));
                [eventAsset,  asset] = obj.checkAssetCollision(UASPos(end, :), obj.UAS.speed*obj.dt);
                [eventNFZ] = obj.checkNFZCollision(UASPos(end, :));
                [eventExitBounds] = obj.checkOutOfBounds(UASPos(end, :), obj.map.size);
                

                if eventSensor == 1 % UAS sensed
                    UASSensedPos = cat(1, UASSensedPos, [obj.tick*obj.tps/60, UASPos(end, :)]);
                    UASSensed = 1;
                    if obj.animate
                        obj.map.updateSensedLocations(UASSensedPos(:, 2:3))
                    end
                end

                if eventAsset == 1 % Asset attacked
                    if ~any(destroyedAssets == asset)
                        destroyedAssets(end + 1) = asset;
                        if obj.animate
                            obj.map.animateDestroyedAssets(obj.assets, destroyedAssets);
                        end
                            lastTick = false;
                    end
                end

                if eventNFZ == 1 % UAS entered NFZ
                    if obj.animate
                        obj.map.animateUASDestroyed(UASPos(end, :))
                    end
                    NFZEntered = true;
                    lastTick = true;
                end

                if eventExitBounds == true % UAS Left the map
                    lastTick = true;
                end

                % Determine UAS Track

                % Determine if UAS can be destroyed
                
                % Update Animation
                if obj.animate
                    pause(obj.dt/obj.animationMultiplier)
                    obj.map.updateUASAnimation(UASPos)
                    if obj.hideClock == false
                        time = obj.tick/obj.tps;
                        obj.map.updateClock(time)
                    end
                end

                obj.tick = obj.tick + 1; % Progress time
            end

            % Clean Sim
            

            % Prepare Results
            results.UASPos = UASPos;
            results.destroyedAssets = destroyedAssets; % Initialize assets destroyed
            results.UASSensed = UASSensed; % Initialize UAS sensed count
            results.UASSensedPos = UASSensedPos;
            results.NFZEntered = NFZEntered; % Initialize NFZ entry status
            results.tick = obj.tick;
        end

        function [event, sensor] = checkSensorCollision(obj, pos)
            % Sensor collision detection
            event = 0; % Initialize event to no collision
            sensor = 0; % Initialize sensor index

            for i = 1:length(obj.sensors)
                r = [pos(1), pos(2)] - obj.sensors(i).location;
                if norm(r) <= obj.sensors(i).range
                    event = 1; % Collision detected
                    sensor = i; % Store the index of the colliding sensor
                    return; % Exit the function early
                end
            end
        end

        function [event, asset] = checkAssetCollision(obj, pos, deltaPos)
            % Asset collision
            for i = 1:length(obj.assets)
                deltaAssetPos = norm(obj.assets(i).location - [pos(1), pos(2)]);
                if deltaAssetPos <= deltaPos
                    event = 1;
                    asset = i;
                    return
                else
                    event = 0; asset = 0;
                end
            end
        end

        function [event, NFZ] = checkNFZCollision(obj, pos)
            % NFZ collision
            event = 0; NFZ = 0;
            if isempty(obj.NFZs) == 0
                for i = 1:length(obj.NFZs)
                    if isinterior(obj.NFZs(i), pos(1), pos(2)) == 1
                        event = 1;
                        NFZ = i;
                        return
                    end
                end
            end
        end

        function [event] = checkOutOfBounds(~, pos, size)
            % Check is UAS is out-of-bounds
            if pos(1) < 0 || pos(1) > size.vert
                event = true;
            elseif pos(2) < 0 || pos(2) > size.horiz
                event = true;
            else
                event = false;
            end
        end
    end
end