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
            obj.targetUnitVector = [dir2D/norm(dir2D), 0];

            obj.active = true;
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

                    obj.planner = plannerHybridAStar( ...
                        sv,'MinTurningRadius',turnRadius, ...
                        "InterpolationDistance",obj.speed*time);

                    refPath = plan(obj.planner, ...
                        [obj.position(1:2), obj.heading], ...
                        [obj.target, obj.heading+pi/2]);

                    obj.pathPoints = refPath.States(:,1:2);
                    obj.pathHeadings = refPath.States(:,3);
                end

                if tick-obj.tickOffset ~= 0
                    if (tick-obj.tickOffset) <= length(obj.pathPoints(:,1))

                        pose = obj.pathPoints(tick-obj.tickOffset,:);
                        obj.position = [pose(1),pose(2),obj.position(3)];
                        obj.heading = obj.pathHeadings(tick-obj.tickOffset);

                    else

                        positionxy = [obj.position(1), obj.position(2)];

                        posEsc = [ ...
                            costMap.XWorldLimits(1), obj.position(2)
                            obj.position(1), costMap.YWorldLimits(1)
                            costMap.XWorldLimits(2), obj.position(2)
                            obj.position(1), costMap.YWorldLimits(2)];

                        [~,Iesc] = min(sum((posEsc-positionxy).^2,2));
                        obj.target = posEsc(Iesc,:);

                        refPath = plan(obj.planner, ...
                            [positionxy,obj.heading], ...
                            [obj.target,obj.heading]);

                        obj.pathPoints = refPath.States(:,1:2);
                        obj.pathHeadings = refPath.States(:,3);

                        obj.tickOffset = tick-1;

                        pose = obj.pathPoints(tick-obj.tickOffset,:);
                        obj.position = [pose(1),pose(2),obj.position(3)];
                    end
                end
            end
        end
    end
end
