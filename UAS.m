classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        range
        active
        destroyedAsset
        totalAsset
        tempSpeed
        altitude
        turnRadius

        adversary_path
        pathPoints
        pathHeadings
        tickOffset
        planner
        heading
    end

    methods
        function obj = UAS(speed, entrance, target, mode, altitude, turnRadius, options)
            arguments
                speed, entrance, target, mode, altitude, turnRadius
                options.adversary_path = [];
                options.searchRange (1,1) double = 20;
            end
            obj.speed = speed;
            obj.altitude = altitude;
            obj.turnRadius = turnRadius;
            obj.position = [entrance(1), entrance(2), altitude];
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            obj.range = options.searchRange;
            
            dir2D = obj.target(1:2) - obj.position(1:2);
            nDir = norm(dir2D);
            if nDir > 0
                obj.targetUnitVector = [dir2D/nDir, 0]; 
            else
                obj.targetUnitVector = [1, 0, 0]; 
            end
            
            obj.active = true; 

            obj.adversary_path = options.adversary_path; 
            obj.pathPoints = [];
            obj.tickOffset = 0;
            obj.planner = [];
            obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));

            if ~isempty(obj.adversary_path)
                obj.pathPoints = obj.adversary_path(:,1:2);
                obj.pathHeadings = obj.adversary_path(:,3);
            end
        end

        function linearMotion(obj, time)
            if obj.active
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

                obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
            end
        end
        
        function hybridAStarMotion(obj, time, tick, turnRadius, costMap)
            if obj.active
                if isempty(obj.planner) && isempty(obj.adversary_path) 
                    ss = stateSpaceSE2;
                    ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                    sv = validatorOccupancyMap(ss);
                    sv.Map = costMap;
                    obj.planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, "InterpolationDistance",obj.speed*time);

                    try
                        refPath = plan(obj.planner, [obj.position(1:2), obj.heading], [obj.target, obj.heading+pi/2]);
                        if ~isempty(refPath.States)
                            obj.pathPoints = refPath.States(:, 1:2); 
                            obj.pathHeadings = refPath.States(:,3);
                        end
                    catch
                        obj.pathPoints = [];
                        obj.pathHeadings = [];
                    end

                    fprintf("WARNING: Generating paths in-loop. Consider pre-generating for speed.\n")
                end
    
                if tick-obj.tickOffset~=0
                    if ~isempty(obj.pathPoints) && (tick - obj.tickOffset) <= length(obj.pathPoints(:,1))
                        pose = obj.pathPoints(tick - obj.tickOffset,:);
                        obj.position = [pose(1), pose(2), obj.position(3)];

                        if ~isempty(obj.pathHeadings)
                            obj.heading = obj.pathHeadings(tick - obj.tickOffset);
                        end

                        obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                        
                    elseif isempty(obj.adversary_path) 
                        positionxy = [obj.position(1), obj.position(2)];
                        posEsc = [costMap.XWorldLimits(1), obj.position(2); obj.position(1), costMap.YWorldLimits(1); costMap.XWorldLimits(2), obj.position(2); obj.position(1), costMap.YWorldLimits(2)];
                        [~, Iesc] = min(sum((posEsc - positionxy).^2, 2));
                        obj.target = posEsc(Iesc, :);

                        try
                            refPath = plan(obj.planner, [positionxy, obj.heading], [obj.target, obj.heading]);
                            if ~isempty(refPath.States)
                                obj.pathPoints = refPath.States(:, 1:2);  
                                obj.tickOffset = tick-1;
                                pose = obj.pathPoints(tick - obj.tickOffset,:);
                                obj.position = [pose(1), pose(2), obj.position(3)];
                                obj.pathHeadings = refPath.States(:,3);
                                obj.heading = obj.pathHeadings(tick - obj.tickOffset);
                            end
                        catch
                            obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
                        end
                        
                    else
                        obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                        obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
                    end
                end
            end
        end

        function searchMotion(obj, time, asset, isAssetDestroyed)
            if ~obj.active
                return;
            end

            if ~isAssetDestroyed
                dist2D = norm(obj.position(1:2) - asset.location(1:2));
                
                if dist2D <= obj.range
                    obj.assetFound(dist2D, asset.location, time);
                else
                    obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;

                    obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
                    obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                end
            end
        end

        function assetFound(obj, assetDistance, assetLocation, time)
            turnRadius = assetDistance/2;
            
            assetVector = [assetLocation, 0] - [obj.position(1:2), 0];
            tuv = [obj.targetUnitVector(1:2), 0];
            
            denom = norm(tuv)*norm(assetVector);
            if denom <= eps
                obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
                return
            end
            
            dotProd = dot(tuv, assetVector)/denom;
            dotProd = max(-1, min(1, dotProd));
            turnAngle = acos(dotProd);
            
            rotDir = cross(tuv, assetVector);
            
            angleVelo = (sin(turnAngle)*obj.speed)/turnRadius;
            angle = angleVelo*time;

            if abs(turnAngle) > 0.1
                obj.turnMotion(angle, rotDir, time);
            else
                obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
            end
        end

        function turnMotion(obj, angle, rotDir, time)
            if rotDir(3) < 0
                DCM = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            elseif rotDir(3) > 0
                DCM = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
            else
                DCM = eye(2);
            end
            
            newVec2D = (DCM * obj.targetUnitVector(1:2)')';
            obj.targetUnitVector = [newVec2D, 0];

            if norm(obj.targetUnitVector(1:2)) > 0
                obj.targetUnitVector(1:2) = obj.targetUnitVector(1:2)/norm(obj.targetUnitVector(1:2));
            end
            obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
            
            obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
        end
    end
end