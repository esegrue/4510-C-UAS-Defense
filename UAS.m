classdef UAS < handle
% Inputs
%
% Output
%
% Notes
% Adversary motion model supporting linear motion, Hybrid A* planning, and search behavior
% Constructor supports UAS(speed, start, target) and UAS(speed, entrance, target, mode, altitude)
% location is a 2-D alias for position(1:2)
% intercept sets active false

properties
    speed (1, 1) double = 0  % forward speed (m/s)
    target (1, 2) double = [0 0]  % target location [x y]
    mode (1, 1) string = "Linear"  % motion mode string
    position (1, 3) double = [0 0 0]  % state position [x y z]
    targetUnitVector (1, 3) double = [1 0 0]  % unit direction vector toward target
    range (1, 1) double = 20  % look-ahead or search range (m)
    active (1, 1) logical = true  % active flag
    obstacles = struct('NFZs', [], 'assets', [])  % obstacle containers
    destroyedAssets = []  % destroyed asset indices
    totalAssets (1, 1) double = 0  % total assets count
    tempSpeed (1, 1) double = 0  % speed used during turning behavior
    altitude (1, 1) double = 0  % altitude (z)

    pathPoints double = []  % planned XY path points
    pathHeadings double = []  % planned headings for path points
    tickOffset (1, 1) double = 0  % tick offset for indexing planned path
    planner = []  % plannerHybridAStar object
    heading (1, 1) double = 0  % heading angle (rad)
end

properties (Dependent)
    location  % alias for position(1:2)
end

methods
    function obj = UAS(speed, entranceOrStart, target, mode, altitude)
        if nargin >= 1 && ~isempty(speed)
            obj.speed = speed;
            obj.tempSpeed = speed;
        end

        if nargin >= 2 && ~isempty(entranceOrStart)
            v = entranceOrStart(:).';
            if numel(v) == 2
                obj.position = [v(1) v(2) obj.altitude];
            elseif numel(v) >= 3
                obj.position = [v(1) v(2) v(3)];
                obj.altitude = v(3);
            else
                error('UAS:BadStart', 'Start or entrance must be a 1x2 or 1x3 vector.');
            end
        end

        if nargin >= 5 && ~isempty(altitude)
            obj.altitude = altitude;
            obj.position(3) = altitude;
        end

        if nargin >= 3 && ~isempty(target)
            obj.target = reshape(target, 1, 2);
        end

        if nargin >= 4 && ~isempty(mode)
            obj.mode = string(mode);
        end

        obj.updateDirectionFromTarget();
        obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));

        obj.pathPoints = [];
        obj.pathHeadings = [];
        obj.tickOffset = 0;
        obj.planner = [];
    end

    function loc = get.location(obj)
        loc = obj.position(1:2);
    end

    function set.location(obj, loc)
        validateattributes(loc, {'double'}, {'vector', 'numel', 2});
        obj.position(1:2) = reshape(loc, 1, 2);
        obj.updateDirectionFromTarget();
        obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
    end

    function setTarget(obj, tgt)
        validateattributes(tgt, {'double'}, {'vector', 'numel', 2});
        obj.target = reshape(tgt, 1, 2);
        obj.updateDirectionFromTarget();
        obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
    end

    function move(obj, dt)
        if ~obj.active
            return
        end
        obj.position = obj.position + obj.speed * dt * obj.targetUnitVector;
    end

    function linearMotion(obj, time)
        if obj.active
            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
        end
    end

    function hybridAStarMotion(obj, time, tick, turnRadius, costMap)
        if ~obj.active
            return
        end

        if isempty(obj.planner)
            ss = stateSpaceSE2;
            ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];

            sv = validatorOccupancyMap(ss);
            sv.Map = costMap;

            obj.planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, "InterpolationDistance", obj.speed * time);

            refPath = plan(obj.planner, [obj.position(1:2), obj.heading], [obj.target, obj.heading + pi / 2]);
            obj.pathPoints = refPath.States(:, 1:2);
            obj.pathHeadings = refPath.States(:, 3);
        end

        if (tick - obj.tickOffset) ~= 0
            idx = tick - obj.tickOffset;

            if idx <= size(obj.pathPoints, 1)
                pose = obj.pathPoints(idx, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading = obj.pathHeadings(idx);
            else
                positionxy = [obj.position(1), obj.position(2)];
                posEsc = [costMap.XWorldLimits(1), obj.position(2); obj.position(1), costMap.YWorldLimits(1); costMap.XWorldLimits(2), obj.position(2); obj.position(1), costMap.YWorldLimits(2)];
                [~, Iesc] = min(sum(posEsc - positionxy, 2), [], "ComparisonMethod", "abs");
                obj.target = posEsc(Iesc, :);

                refPath = plan(obj.planner, [positionxy, obj.heading], [obj.target, obj.heading]);
                obj.pathPoints = refPath.States(:, 1:2);
                obj.pathHeadings = refPath.States(:, 3);

                obj.tickOffset = tick - 1;

                pose = obj.pathPoints(1, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading = obj.pathHeadings(1);
            end
        end

        obj.updateDirectionFromTarget();
    end

    function searchMotion(obj, time, assets, destroyedAssets, NFZs)
        if ~obj.active
            return
        end

        if nargin < 3
            assets = [];
        end
        if nargin < 4
            destroyedAssets = [];
        end
        if nargin < 5
            NFZs = [];
        end

        obj.range = 20;
        obj.obstacles.NFZs = NFZs;
        obj.obstacles.assets = assets;
        obj.totalAssets = numel(assets);
        obj.destroyedAssets = destroyedAssets;

        if ~isempty(NFZs)
            obj.avoidNFZ();
        end

        if isempty(assets)
            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
            return
        end

        aliveIdx = setdiff(1:numel(assets), destroyedAssets);
        if isempty(aliveIdx)
            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
            return
        end

        locs = zeros(numel(aliveIdx), 2);
        for k = 1:numel(aliveIdx)
            idx = aliveIdx(k);

            if iscell(assets)
                a = assets{idx};
            else
                a = assets(idx);
            end

            if isstruct(a)
                if isfield(a, 'location')
                    loc = a.location;
                elseif isfield(a, 'position')
                    loc = a.position;
                else
                    error('UAS:AssetNoLocation', 'Struct asset at index %d has no location or position field.', idx);
                end
            else
                if isprop(a, 'location')
                    loc = a.location;
                elseif isprop(a, 'position')
                    loc = a.position;
                else
                    error('UAS:AssetNoLocation', 'Asset at index %d has no location or position property.', idx);
                end
            end

            loc = loc(:).';
            if numel(loc) < 2
                error('UAS:AssetBadLocation', 'Asset location at index %d must have at least 2 elements.', idx);
            end
            locs(k, :) = loc(1:2);
        end

        d = locs - obj.position(1:2);
        dist = vecnorm(d, 2, 2);
        [minDist, kmin] = min(dist);

        if minDist <= obj.range
            obj.assetFound(minDist, kmin, time, locs);
        else
            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
        end
    end

    function assetFound(obj, assetDistance, assetNumber, time, assetLocations)
        turnRadius = assetDistance / 2;

        assetLocationVec = [assetLocations(assetNumber, :) 0] - [obj.position(1:2) 0];
        tuv = obj.targetUnitVector;

        denom = norm(tuv) * norm(assetLocationVec);
        if denom <= 0
            obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
            return
        end

        cosang = dot(tuv, assetLocationVec) / denom;
        cosang = max(-1, min(1, cosang));
        turnAngle = acos(cosang);
        rotDir = cross(tuv, assetLocationVec);

        angleVelo = (sin(turnAngle) * obj.speed) / max(turnRadius, eps);
        angle = angleVelo * time;

        if abs(turnAngle) > 0.1
            obj.turnMotion(angle, rotDir, time);
        else
            obj.position = obj.position + obj.tempSpeed * time * obj.targetUnitVector;
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
        n = norm(newVec2D);
        if n > 0
            newVec2D = newVec2D / n;
        end
        obj.targetUnitVector = [newVec2D, 0];

        obj.position = obj.position + obj.tempSpeed * time * obj.targetUnitVector;
        obj.heading = atan2(obj.targetUnitVector(2), obj.targetUnitVector(1));
    end

    function avoidNFZ(obj)
        if isempty(obj.obstacles) || ~isfield(obj.obstacles, 'NFZs') || isempty(obj.obstacles.NFZs)
            return
        end

        NFZ = obj.obstacles.NFZs;
        if numel(NFZ) > 1
            try
                P = NFZ(1);
                for k = 2:numel(NFZ)
                    P = union(P, NFZ(k));
                end
                NFZ = P;
            catch
                NFZ = NFZ(1);
            end
        end

        pCheck = obj.position + obj.targetUnitVector * obj.range;

        if isinterior(NFZ, pCheck(1), pCheck(2))
            angle = linspace(-pi / 4, pi / 4, 100);
            options = zeros(100, 2);
            for n = 1:length(angle)
                R = [cos(-angle(n)) -sin(-angle(n)); sin(-angle(n)) cos(-angle(n))];
                check = obj.position(1:2)' + R * (obj.targetUnitVector(1:2)' * obj.range);
                options(n, :) = check';
            end

            crash = find(isinterior(NFZ, options(:, 1), options(:, 2)) == true); %#ok<NASGU,FNDSB>
        end
    end

    function intercept(obj)
        obj.active = false;
    end
end

methods (Access = private)
    function updateDirectionFromTarget(obj)
        d = obj.target - obj.position(1:2);
        n = norm(d);
        if n > 0
            obj.targetUnitVector = [d / n, 0];
        end
    end
end
end
