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
            end
            obj.speed = speed;
            obj.altitude = altitude;
            obj.turnRadius = turnRadius;
            obj.position = [entrance(1), entrance(2), altitude];
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            
            % Safe vector normalization
            dir2D = obj.target(1:2) - obj.position(1:2);
            nDir = norm(dir2D);
            if nDir > 0
                obj.targetUnitVector = [dir2D/nDir, 0]; 
            else
                obj.targetUnitVector = [1, 0, 0]; 
            end
            
            obj.active = true; % Default to active

            obj.adversary_path = options.adversary_path; %please note that this will have both ingress and egress
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
                    refPath = plan(obj.planner, [obj.position(1:2), obj.heading], [obj.target, obj.heading+pi/2]);
                    obj.pathPoints = refPath.States(:, 1:2); 
                    obj.pathHeadings = refPath.States(:,3);
                    fprintf("WARNING: Generating paths in-loop. Consider pre-generating for speed.\n")
                end
    
                if tick-obj.tickOffset~=0
                    if (tick - obj.tickOffset) <= length(obj.pathPoints(:,1))
                        pose = obj.pathPoints(tick - obj.tickOffset,:);
                        obj.position = [pose(1), pose(2), obj.position(3)];
                        obj.heading = obj.pathHeadings(tick - obj.tickOffset);
                        
                    elseif isempty(obj.adversary_path) 
                        positionxy = [obj.position(1), obj.position(2)];
                        posEsc = [costMap.XWorldLimits(1), obj.position(2); obj.position(1), costMap.YWorldLimits(1); costMap.XWorldLimits(2), obj.position(2); obj.position(1), costMap.YWorldLimits(2)];
                        [~, Iesc] = min(sum((posEsc - positionxy).^2, 2));
                        obj.target = posEsc(Iesc, :);
                        refPath = plan(obj.planner, [positionxy, obj.heading], [obj.target, obj.heading]);
                        obj.pathPoints = refPath.States(:, 1:2);  
                        obj.tickOffset = tick-1;
                        pose = obj.pathPoints(tick - obj.tickOffset,:);
                        obj.position = [pose(1), pose(2), obj.position(3)];
                        obj.pathHeadings = refPath.States(:,3);
                        
                    else
                        % BUG FIX: Coast straight forward if pre-planned points run out
                        obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                        obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
                    end
                else
                    obj.position = obj.position;
                end
            end
        end

        function searchMotion(obj, time, asset, isAssetDestroyed)
            if ~obj.active
                return;
            end
            
            obj.range = 20;

            if ~isAssetDestroyed
                dist2D = norm(obj.position(1:2) - asset.location(1:2));
                
                if dist2D <= obj.range
                    obj.assetFound(dist2D, asset.location, time);
                else
                    obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
                end
            end
        end

        function assetFound(obj, assetDistance, assetLocation, time)
            turnRadius = assetDistance/2;
            
            assetVector = [assetLocation, 0] - [obj.position(1:2), 0];
            tuv = [obj.targetUnitVector(1:2), 0];
            
            % Safe ACOS calculation to prevent complex numbers
            dotProd = dot(tuv, assetVector)/(norm(tuv)*norm(assetVector));
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
            
            obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
        end
    end
end