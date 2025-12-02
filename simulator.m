classdef simulator
    % SIMULATOR for C-UAS Effector Placement
    
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
        fadePings 
    end

    methods
        function obj = simulator(map, aor, uas, effectors, sensors, assets, options)
            arguments
                map
                aor
                uas
                effectors
                sensors
                assets
                options.tps double = 20
                options.animate logical = true
                options.nfzs polyshape = polyshape.empty
                options.resetGraphics logical = true
                options.animationMultiplier double = 1
                options.hideClock logical = false
                options.fadePings logical = false 
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
            obj.fadePings = options.fadePings; 
            
            % Initialize history for N UAS
            obj.UASPos_all = cell(1, length(obj.UAS));
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = [obj.UAS(i).position(1), obj.UAS(i).position(2)];
            end
        end

        function results = runSim(obj)
            destroyedAssets = [];
            cost = 0;
            UASkilled = 0;
            UASsensed = 0;
            NFZEntered = false;
            simComplete = false;
            
            UASsensedPos = [];
            UASkilledPos = [];
            
            % Initialize Graphics
            if obj.animate
                if obj.resetGraphics
                    obj.map.wipeAnimation()
                end
                % Pass number of UAS to map
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.effectors, obj.sensors, length(obj.UAS), obj.hideClock);
            end

            req_pings = obj.sensors(1).params.pings;
            window_duration = obj.sensors(1).params.duration;
            window_ticks = ceil(window_duration/obj.dt);
            track_hist = zeros(length(obj.UAS), window_ticks);

            while ~simComplete
                simComplete = true; 
                
                % 1. MOVE ALL UAS
                if obj.tick ~= 0
                    for i = 1:length(obj.UAS)
                        if obj.UAS(i).active
                            simComplete = false; 
                            if obj.UAS(i).mode == "Linear"
                                obj.UAS(i).linearMotion(obj.dt);
                            elseif obj.UAS(i).mode == "Search"
                                obj.UAS(i).searchMotion(obj.dt, obj.assets, destroyedAssets, obj.NFZs);
                            end
                            obj.UASPos_all{i} = cat(1, obj.UASPos_all{i}, obj.UAS(i).position);
                        end
                    end
                else
                    simComplete = false;
                end

                if simComplete
                    break; 
                end

                % 2. CHECK COLLISIONS
                for i = 1:length(obj.UAS)
                    if ~obj.UAS(i).active
                        continue;
                    end
                    
                    pos = obj.UAS(i).position;
                    currentTime = obj.tick * obj.dt; 
                    
                    [isPinged, isTracked, track_hist(i,:)] = obj.checkSensorCollision(pos, track_hist(i,:), req_pings);
                    [eventEffector] = obj.checkEffectorCollision(pos, isTracked);
                    [eventAsset, assetID] = obj.checkAssetCollision(pos, obj.UAS(i).speed * obj.dt);
                    [eventNFZ] = obj.checkNFZCollision(pos);
                    [eventExitBounds] = obj.checkOutOfBounds(pos, obj.map.size);

                    if isPinged
                         UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                         UASsensed = UASsensed + 1; 
                         
                         if obj.animate
                             if obj.fadePings
                                 validIdx = UASsensedPos(:,1) >= (currentTime - window_duration);
                                 plotData = UASsensedPos(validIdx, :);
                                 obj.map.animateUASsensed(plotData);
                             else
                                 obj.map.animateUASsensed(UASsensedPos);
                             end
                         end
                    end

                    if eventEffector
                        UASkilledPos = cat(1, UASkilledPos, [currentTime, pos]);
                        UASkilled = UASkilled + 1;
                        cost = cost + 100;
                        obj.UAS(i).active = false; 
                        if obj.animate
                            obj.map.animateUASkilled(pos)
                        end
                    end

                    if eventAsset
                        if ~any(destroyedAssets == assetID)
                            destroyedAssets(end + 1) = assetID;
                            cost = cost + 1000;
                            if obj.animate
                                obj.map.animateDestroyedAssets(obj.assets, destroyedAssets);
                            end
                            simComplete = true; 
                        end
                    end

                    if eventNFZ
                        if obj.animate
                            obj.map.animateUASkilled(pos) 
                        end
                        NFZEntered = true;
                        obj.UAS(i).active = false;
                    end

                    if eventExitBounds
                        obj.UAS(i).active = false;
                    end
                end

                % 3. UPDATE ANIMATION
                if obj.animate
                    pause(obj.dt/obj.animationMultiplier)
                    % Pass entire cell array to map
                    if ~isempty(obj.UASPos_all)
                         obj.map.updateUASAnimation(obj.UASPos_all); 
                    end
                    if ~obj.hideClock
                        time = obj.tick/obj.tps;
                        obj.map.updateClock(time)
                    end
                end

                obj.tick = obj.tick + 1;
            end

            results.UASPos_all = obj.UASPos_all;
            results.destroyedAssets = destroyedAssets; 
            results.cost = cost; 
            results.UASkilled = UASkilled;
            results.UASkilledPos = UASkilledPos;
            results.NFZEntered = NFZEntered; 
            results.tick = obj.tick;
        end

        function [isPinged, isTracked, new_hist] = checkSensorCollision(obj, pos, current_hist, req_pings)
            if isempty(obj.sensors)
                isPinged = 0; isTracked = 0; new_hist = current_hist; return;
            end
            
            probs = zeros(1, length(obj.sensors));
            for k = 1:length(obj.sensors)
                probs(k) = interp2(obj.sensors(k).xg, obj.sensors(k).yg, obj.sensors(k).P, pos(1), pos(2), 'linear', 0);
            end
            
            max_detect_prob = max(probs);
            isPinged = (max_detect_prob >= rand());
            new_hist = [current_hist(2:end), isPinged];
            isTracked = (sum(new_hist) >= req_pings);
        end

        function [event, effectorID] = checkEffectorCollision(obj, pos, isTracked)
            event = 0; effectorID = 0;
            if ~isTracked || isempty(obj.effectors)
                return;
            end
            
            effLocs = reshape([obj.effectors.location], 2, [])';
            effRanges = [obj.effectors.range]';
            dists = sqrt(sum((effLocs - pos).^2, 2));
            inRangeIdx = find(dists <= effRanges);
            
            if ~isempty(inRangeIdx)
                event = 1;
                effectorID = inRangeIdx(1); 
            end
        end

        function [event, assetID] = checkAssetCollision(obj, pos, deltaPos)
            event = 0; assetID = 0;
            if isempty(obj.assets); return; end
            
            assetLocs = reshape([obj.assets.location], 2, [])';
            dists = sqrt(sum((assetLocs - pos).^2, 2));
            hitIdx = find(dists <= deltaPos);
            if ~isempty(hitIdx)
                event = 1;
                assetID = hitIdx(1);
            end
        end

        function [event, NFZID] = checkNFZCollision(obj, pos)
            event = 0; NFZID = 0;
            if isempty(obj.NFZs); return; end
            for i = 1:length(obj.NFZs)
                if isinterior(obj.NFZs(i), pos(1), pos(2))
                    event = 1; NFZID = i; return;
                end
            end
        end

        function [event] = checkOutOfBounds(~, pos, size)
            event = (pos(1) < 0 || pos(1) > size.vert || pos(2) < 0 || pos(2) > size.horiz);
        end
    end
end