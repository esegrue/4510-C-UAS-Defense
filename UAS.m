classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        range

        obstacles
        destroyedAssets
        totalAssets
        tempSpeed


        pathPoints
        pathHeadings
        tickOffset
        planner
        heading

    end

    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.position = entrance;
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            obj.targetUnitVector = (obj.target - obj.position)/norm(obj.target - obj.position);

            obj.pathPoints = [];
            obj.tickOffset = 0;
            obj.planner = [];
            obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));

        end

        function obj = linearMotion(obj,time)
            obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

        end

        function obj = hybridAStarMotion(obj, time, tick, turnRadius, costMap)
            if isempty(obj.planner)
                ss = stateSpaceSE2;
                ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                sv = validatorOccupancyMap(ss);
                sv.Map = costMap;
                obj.planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, 'ReverseCost', 100000, 'DirectionSwitchingCost', 100000, "InterpolationDistance",obj.speed*time);
                % Plan initial path
                refPath = plan(obj.planner, [obj.position, obj.heading], [obj.target, 0]);
                obj.pathPoints = refPath.States(:, 1:2);  % Just x,y coordinates
                obj.pathHeadings = refPath.States(:,3);
                pts = refPath.States; % [x y theta]
            end



            if (tick - obj.tickOffset) <= length(obj.pathPoints(:,1))
                pose = obj.pathPoints(tick - obj.tickOffset,:);
                obj.position = pose;
                obj.heading = obj.pathHeadings(tick - obj.tickOffset);
            else %reached end of path, now escape
                posEsc = [costMap.XWorldLimits(1), obj.position(2); obj.position(1), costMap.YWorldLimits(1); costMap.XWorldLimits(2), obj.position(2); obj.position(1), costMap.YWorldLimits(2)];
                [~, Iesc] = min(sum(posEsc - obj.position, 2), [], "ComparisonMethod", "abs");
                obj.target = posEsc(Iesc, :);
                refPath = plan(obj.planner, [obj.position, obj.heading], [obj.target, obj.heading]);
                obj.pathPoints = refPath.States(:, 1:2);  % Just x,y coordinates
                pts = refPath.States; % [x y theta]
                plot(pts(:,1), pts(:,2), 'g--', 'LineWidth', 2);
                obj.tickOffset = tick - 1;
                obj.position = obj.pathPoints(tick - obj.tickOffset,:);
            end
            obj.position
        end

        function obj = searchMotion(obj,time,assets,destroyedAssets,NFZs)
            obj.range = 20;
            obj.obstacles.NFZs = NFZs;
            obj.obstacles.assets = assets;
            obj.avoidNFZ

            obj.totalAssets = length(obj.obstacles.assets);
            obj.destroyedAssets = destroyedAssets;

            if ~isempty(obj.destroyedAssets)
                obj.obstacles.assets(obj.destroyedAssets) = [];

            end

            if ~isempty(obj.obstacles.assets)
                for n = 1:length(obj.obstacles.assets)
                    assetDistance(n) = norm(obj.position - obj.obstacles.assets(n).location);

                end
                [assetDistance, assetNumber] = min(assetDistance);

                if assetDistance <= obj.range
                    obj.assetFound(assetDistance,assetNumber,time)

                else
                    obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

                end
            end
        end

        function assetFound(obj,assetDistance,assetNumber,time)
            turnRadius = assetDistance/2;
            assetLocation = obj.obstacles.assets(assetNumber).location - obj.position;
            turnAngle = acos(dot(obj.targetUnitVector,assetLocation)/(norm(obj.targetUnitVector)*norm(assetLocation)));
            rotDir = cross([obj.targetUnitVector,0],[obj.position,0]-[obj.obstacles.assets(assetNumber).location,0]);
            angleVelo = (sin(turnAngle)*obj.speed)/turnRadius;

            % if angleVelo > 1
            %     angleVelo = 1;
            % 
            % end

            angle = angleVelo*time;

            if abs(turnAngle) > 0.1
                obj.turnMotion(angle,rotDir,time)

            else
                obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
            end
        end

        function turnMotion(obj,angle,rotDir,time)
            if rotDir(3) < 0
                DCM = [cos(angle) -sin(angle);sin(angle) cos(angle)];
            elseif rotDir(3) > 0
                DCM = [cos(-angle) -sin(-angle);sin(-angle) cos(-angle)];
            end
            obj.targetUnitVector = (DCM*obj.targetUnitVector')';

            obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;

        end

        function avoidNFZ(obj)
            if isinterior(obj.obstacles.NFZs,obj.position + obj.targetUnitVector*obj.range)
                angle = linspace(-pi/4,pi/4,100);
                for n = 1:length(angle)
                    check = obj.position' + [cos(-angle(n)) -sin(-angle(n));sin(-angle(n)) cos(-angle(n))]*obj.targetUnitVector'*obj.range;
                    options(n,:) = check';

                end
                crash = find(isinterior(obj.obstacles.NFZs,options)== true)

            end

        end
    end
end
