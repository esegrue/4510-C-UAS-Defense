classdef simulator
    %SIMULATOR for C-UAS Effector Placement
    %   This is the main simulator class for the UAS-Effector Placement. This
    %   is where time will march, and from here all map updates are called.

    properties
        map
        AOR
        UAS
        UASPos_all
        effectors
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
        cost
    end

    methods
        function obj = simulator(map, aor, uas, effectors, sensors, assets, options)
            %SIMULATOR
            arguments
                map
                aor
                uas
                effectors
                sensors
                assets
                options.tps double = 20                                     % How many ticks per second the logical system should operate in, AKA: simulation resolution
                options.animate logical = true                              % Set false if you just want data
                options.nfzs polyshape = polyshape.empty                    % This is where you declare any NFZs you want, should you decide to do so
                options.resetGraphics logical = true                        % This will reset all graphics if used over multiple iterations FOR THE SAME MAP! As such, default is true
                options.animationMultiplier double = 1                      % Animation speed multiplier, default 1x
                options.hideClock logical = false                           % Set true if you want to hide the clock
            end
            obj.map = map;
            obj.AOR = aor;
            obj.UAS = uas;
            obj.effectors = effectors;
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
            destroyedAssets = [];                                           % Initialize assets destroyed
            cost = 0;
            UASkilled = 0;                                                  % Initialize UAS killed count
            UASsensed = 0;
            NFZEntered = false;                                             % Initialize NFZ entry status
            lastTick = false;                                               % Initialize lastTick to be set true when simulation should end
            UASsensedPos = [];
            UASkilledPos = [];                                              % Initialize matrix to track all positions in which the UAS is killed
            UASPos_all = [obj.UAS.position(1), obj.UAS.position(2)];            % This matrix tracks all current and previous UAS positions
            
            if obj.animate == true
                if obj.resetGraphics
                    obj.map.wipeAnimation()
                end
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.effectors, obj.sensors, obj.hideClock);
            end

            req_pings = obj.sensors(1).params.pings;
            window_duration = obj.sensors(1).params.duration;
            window_ticks = ceil(window_duration/obj.dt);
            track_hist = zeros([1 window_ticks]);

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
                    UASPos_all = cat(1, UASPos_all, obj.UAS.position);
                end
                UASPos = UASPos_all(end,:);


                % Check for any logical events
                [eventSensor, ~, ~, track_hist] = obj.checkSensorCollision(UASPos, track_hist);   
                [eventEffector] = obj.checkEffectorCollision(UASPos, eventSensor);
                [eventAsset,  asset] = obj.checkAssetCollision(UASPos, obj.UAS.speed*obj.dt);
                [eventNFZ] = obj.checkNFZCollision(UASPos);
                [eventExitBounds] = obj.checkOutOfBounds(UASPos, obj.map.size);

                if eventSensor == 1
                    UASsensedPos = cat(1, UASsensedPos, [obj.tick*obj.tps/60, UASPos]);
                    UASsensed = 1;
                    if obj.animate
                        obj.map.animateUASsensed(UASsensedPos)
                    end
                end

                if eventEffector == 1 % UAS killed
                    UASkilledPos = cat(1, UASkilledPos, [obj.tick*obj.tps/60, UASPos]);
                    UASkilled = 1;
                    cost = cost + 100; % cost to use effector
                    if obj.animate
                        obj.map.animateUASkilled(UASPos)
                        %obj.map.updatekilledLocations(UASkilledPos(:, 2:3))
                    end
                    lastTick = true;
                end

                if eventAsset == 1 % Asset attacked
                    if ~any(destroyedAssets == asset)
                        destroyedAssets(end + 1) = asset;
                        cost = cost + 1000; % cost of mission failure
                        if obj.animate
                            obj.map.animateDestroyedAssets(obj.assets, destroyedAssets);
                        end
                        lastTick = false;
                    end
                end

                if eventNFZ == 1 % UAS entered NFZ
                    if obj.animate
                        obj.map.animateUASkilled(UASPos)
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
                    obj.map.updateUASAnimation(UASPos_all)
                    if obj.hideClock == false
                        time = obj.tick/obj.tps;
                        obj.map.updateClock(time)
                    end
                end

                obj.tick = obj.tick + 1; % Progress time
            end

            % Clean Sim
            

            % Prepare Results
            results.UASPos_all = UASPos_all;
            results.destroyedAssets = destroyedAssets; % Initialize assets destroyed
            results.cost = cost; % Initialize cost
            results.UASkilled = UASkilled; % Initialize UAS killed count
            results.UASkilledPos = UASkilledPos;
            results.NFZEntered = NFZEntered; % Initialize NFZ entry status
            results.tick = obj.tick;
        end

        function [event, sensorID, ping, track_hist] = checkSensorCollision(obj, pos, track_hist)
            % Sensor collision detection
            event = 0;
            sensorID = [];
            max_detect_prob = 0;
            for i = 1:length(obj.sensors)
                sensor = obj.sensors(i);
                detection_prob(i) = P_at_location(sensor, pos);
            end
            [max_detect_prob, sensorID] = max(detection_prob);
            ping = (max_detect_prob >= rand());
            track_hist = [track_hist(2:end), ping];
            tot_pings = sum(track_hist);
            event = (tot_pings >= obj.sensors(1).params.pings);
        end

        function [event, effectors] = checkEffectorCollision(obj, pos, eventSensor)
            % Effector collision detection
            event = 0; % Initialize event to no collision
            effectors = 0; % Initialize effectors index
            
            if eventSensor
                for i = 1:length(obj.effectors)
                    r = [pos(1), pos(2)] - obj.effectors(i).location;
                    if norm(r) <= obj.effectors(i).range
                        event = 1; % Collision detected
                        effectors = i; % Store the index of the colliding effectors
                        return; % Exit the function early
                    end
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