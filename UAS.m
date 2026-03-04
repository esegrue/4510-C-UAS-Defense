classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        range
        active
        obstacles
        destroyedAsset
        totalAsset
        tempSpeed
        altitude
        turnRadius

        pathPoints
        pathHeadings
        tickOffset
        planner
        heading
    end

    methods
        function obj = UAS(speed, entrance, target, mode, altitude, turnRadius)
            obj.speed = speed;
            obj.altitude = altitude;
            obj.turnRadius = turnRadius;
            obj.position = [entrance(1), entrance(2), altitude];
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            dir2D = obj.target(1:2) - obj.position(1:2);
            obj.targetUnitVector = [dir2D/norm(dir2D), 0]; % Z-component is 0
            obj.active = true; % Default to active


            obj.pathPoints = [];
            obj.tickOffset = 0;
            obj.planner = [];
            obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
        end

        function linearMotion(obj, time)
            if obj.active
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
            end
        end
        function hybridAStarMotion(obj, time, tick, turnRadius, costMap)
            if obj.active
                if isempty(obj.planner)
                    ss = stateSpaceSE2;
                    ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                    sv = validatorOccupancyMap(ss);
                    sv.Map = costMap;
                    %show(costMap)
                    obj.planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, "InterpolationDistance",obj.speed*time);
                    % Plan initial path
                    refPath = plan(obj.planner, [obj.position(1:2), obj.heading], [obj.target, obj.heading+pi/2]);
                    obj.pathPoints = refPath.States(:, 1:2);  % Just x,y coordinates
                    obj.pathHeadings = refPath.States(:,3);
                    pts = refPath.States; % [x y theta]
                end
    
                if tick-obj.tickOffset~=0
                    if (tick - obj.tickOffset) <= length(obj.pathPoints(:,1))
                        pose = obj.pathPoints(tick - obj.tickOffset,:);
                        obj.position = [pose(1), pose(2), obj.position(3)];
                        obj.heading = obj.pathHeadings(tick - obj.tickOffset);
                        
      
                    else %reached end of path, now escape
                        positionxy = [obj.position(1), obj.position(2)];
                        posEsc = [costMap.XWorldLimits(1), obj.position(2); obj.position(1), costMap.YWorldLimits(1); costMap.XWorldLimits(2), obj.position(2); obj.position(1), costMap.YWorldLimits(2)];
                        [~, Iesc] = min(sum((posEsc - positionxy).^2, 2));
                        obj.target = posEsc(Iesc, :);
                        refPath = plan(obj.planner, [positionxy, obj.heading], [obj.target, obj.heading]);
                        obj.pathPoints = refPath.States(:, 1:2);  % Just x,y coordinates
                        pts = refPath.States; % [x y theta]
                        %plot(pts(:,1), pts(:,2), 'g--', 'LineWidth', 2);
                        obj.tickOffset = tick-1;
                        pose = obj.pathPoints(tick - obj.tickOffset,:);
                        obj.position = [pose(1), pose(2), obj.position(3)];
                        obj.pathHeadings = refPath.States(:,3);
                    end
                else
                    obj.position = obj.position;
                end
            end
        end

        function searchMotion(obj, time, asset, isAssetDestroyed, NFZs)
            if ~obj.active
                return;
            end
            
            obj.range = 20;
            obj.obstacles.NFZs = NFZs;
            obj.obstacles.asset = asset;
            
            % avoidNFZ logic needs to be fixed to actually turn the UAS
            obj.avoidNFZ(); 

            if ~isAssetDestroyed
                dist2D = norm(obj.position(1:2) - asset.location(1:2));
                
                if dist2D <= obj.range
                    obj.assetFound(dist2D, asset.location, time);
                else
                    obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
                end
            end
        end

        function assetFound(obj, assetDistance, assetLocation, time)
            turnRadius = assetDistance/2;
            
            assetVector = [assetLocation, 0] - [obj.position(1:2), 0];
            
            tuv = [obj.targetUnitVector(1:2), 0];
            
            turnAngle = acos(dot(tuv, assetVector)/(norm(tuv)*norm(assetVector)));
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

        function avoidNFZ(obj)
            % Check if the look-ahead point is inside an NFZ
            lookAhead = obj.position(1:2) + obj.targetUnitVector(1:2)*obj.range;
            if ~isinterior(obj.obstacles.NFZs, lookAhead)
                return;  % path is clear, nothing to do
            end

            % Sweep candidate angles and pick the smallest deflection that is clear
            angles = linspace(-pi/2, pi/2, 180);
            for n = 1:length(angles)
                R = [cos(angles(n)) -sin(angles(n)); sin(angles(n)) cos(angles(n))];
                candidate = obj.position(1:2)' + R * obj.targetUnitVector(1:2)' * obj.range;
                if ~isinterior(obj.obstacles.NFZs, candidate')
                    newDir = R * obj.targetUnitVector(1:2)';
                    obj.targetUnitVector = [newDir', 0];
                    return;
                end
            end
        end
    end
end