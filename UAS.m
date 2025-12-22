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
        destroyedAssets
        totalAssets
        tempSpeed
        altitude
    end

    methods
        function obj = UAS(speed, entrance, target, mode, altitude)
            obj.speed = speed;
            obj.altitude = altitude;
            obj.position = [entrance(1), entrance(2), altitude];
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            dir2D = obj.target(1:2) - obj.position(1:2);
            obj.targetUnitVector = [dir2D/norm(dir2D), 0]; % Z-component is 0
            obj.active = true; % Default to active
        end

        function linearMotion(obj, time)
            if obj.active
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
            end
        end

        function searchMotion(obj, time, assets, destroyedAssets, NFZs)
            if ~obj.active
                return;
            end
            
            obj.range = 20;
            obj.obstacles.NFZs = NFZs;
            obj.obstacles.assets = assets;
            
            % avoidNFZ logic needs to be fixed to actually turn the UAS
            obj.avoidNFZ(); 

            obj.totalAssets = length(obj.obstacles.assets);
            obj.destroyedAssets = destroyedAssets;

            currentAssets = obj.obstacles.assets;
            if ~isempty(obj.destroyedAssets)
                currentAssets(obj.destroyedAssets) = [];
            end

            if ~isempty(currentAssets)
                assetDistance = zeros(1, length(currentAssets));
                for n = 1:length(currentAssets)
                    dist2D = norm(obj.position(1:2) - currentAssets(n).location(1:2));
                    assetDistance(n) = dist2D;
                end
                [minDist, assetNumber] = min(assetDistance);

                if minDist <= obj.range
                    obj.assetFound(minDist, assetNumber, time, currentAssets);
                else
                    obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
                end
            end
        end

        function assetFound(obj, assetDistance, assetNumber, time, currentAssets)
            turnRadius = assetDistance/2;
            
            assetLocation = [currentAssets(assetNumber).location, 0] - [obj.position(1:2), 0];
            
            tuv = [obj.targetUnitVector(1:2), 0];
            
            turnAngle = acos(dot(tuv, assetLocation)/(norm(tuv)*norm(assetLocation)));
            rotDir = cross(tuv, assetLocation);
            
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
            % detects but does not yet turn
            if isinterior(obj.obstacles.NFZs, obj.position + obj.targetUnitVector*obj.range)
                angle = linspace(-pi/4, pi/4, 100);
                options = zeros(100, 2);
                for n = 1:length(angle)
                    check = obj.position' + [cos(-angle(n)) -sin(-angle(n)); sin(-angle(n)) cos(-angle(n))]*obj.targetUnitVector'*obj.range;
                    options(n, :) = check';
                end
                crash = find(isinterior(obj.obstacles.NFZs, options) == true);
            end
        end
    end
end