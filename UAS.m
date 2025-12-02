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
    end

    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.position = entrance;
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            obj.targetUnitVector = (obj.target - obj.position)/norm(obj.target - obj.position);
            obj.active = true; % Default to active
        end

        function linearMotion(obj, time)
            if obj.active
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
            end
        end

        function searchMotion(obj, time, assets, destroyedAssets, NFZs)
            if ~obj.active; return; end
            
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
                    assetDistance(n) = norm(obj.position - currentAssets(n).location);
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
            assetLocation = currentAssets(assetNumber).location - obj.position;
            turnAngle = acos(dot(obj.targetUnitVector, assetLocation)/(norm(obj.targetUnitVector)*norm(assetLocation)));
            rotDir = cross([obj.targetUnitVector, 0], [obj.position, 0] - [currentAssets(assetNumber).location, 0]);
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
            obj.targetUnitVector = (DCM*obj.targetUnitVector')';
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