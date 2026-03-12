classdef map < handle

    properties
        size
        terrain
        resolution
        terrainProxy

        UASTrail
        UASHead
        UASsensed
        UASkilled
        UAScrashed
        assetDestroyed
        assets
        timeBox

        NFZhandle
        obstacleHandles
        defenderHandles

        effectorHandles
        effectorRingHandles
        sensorHandles
        sensorRingHandles

        figSim
        axSim
        figTerr
        axTerr

        maxTrailPoints (1,1) double = 400

        occMapCache
        occMapCacheThreshold (1,1) double = NaN

        eventBox

        defenderTrailHandles
        defenderTrailHist
        defenderMaxTrailPoints (1,1) double = 400

        projectileHandles

        killFlashHandles
        killFlashExpireTimes
        killFlashDuration (1,1) double = 0.35

        is3DView (1,1) logical = false
    end

    methods

        function obj = map(vertical,horizontal,resolution)

            if nargin < 3
                resolution = 1;
            end

            obj.size.vert = vertical;
            obj.size.horiz = horizontal;
            obj.resolution = resolution;

            obj.generateTerrain("Flat");

            obj.occMapCache = [];
            obj.occMapCacheThreshold = NaN;

            obj.figSim = [];
            obj.axSim = [];
            obj.figTerr = [];
            obj.axTerr = [];

            obj.projectileHandles = gobjects(0);

            obj.killFlashHandles = gobjects(0);
            obj.killFlashExpireTimes = [];

        end


        function generateTerrain(obj,type,varargin)

            xVec = 0:obj.resolution:obj.size.horiz;
            yVec = 0:obj.resolution:obj.size.vert;

            [X,Y] = meshgrid(xVec,yVec);
            Z = zeros(size(X));

            switch type

                case "Flat"

                case "Hills"

                    maxH = 20;
                    if ~isempty(varargin)
                        maxH = varargin{1};
                    end

                    centers = [obj.size.horiz*[0.2 0.7 0.5]; obj.size.vert*[0.3 0.8 0.4]];
                    widths = [15 20 10];
                    heights = [0.8*maxH 1.0*maxH 0.6*maxH];

                    for i = 1:length(widths)
                        Z = Z + heights(i)*exp(-((X-centers(1,i)).^2 + (Y-centers(2,i)).^2)/(2*widths(i)^2));
                    end

            end

            obj.terrain.X = X;
            obj.terrain.Y = Y;
            obj.terrain.Z = Z;

            obj.terrainProxy = griddedInterpolant({yVec,xVec},Z,"linear","nearest");

            obj.occMapCache = [];
            obj.occMapCacheThreshold = NaN;

        end


        function z = getElevation(obj,x,y)

            if isempty(obj.terrainProxy)
                z = zeros(size(x));
            else
                z = obj.terrainProxy(y,x);
            end

        end


        function occ = getOccMap(obj,elevThreshold)

            if nargin < 2
                elevThreshold = 20;
            end

            if ~isempty(obj.occMapCache) && isequal(obj.occMapCacheThreshold,elevThreshold)
                occ = obj.occMapCache;
                return
            end

            Z = obj.terrain.Z;

            costmap = (Z > elevThreshold);

            occ = binaryOccupancyMap(fliplr(costmap));

            obj.occMapCache = occ;
            obj.occMapCacheThreshold = elevThreshold;

        end


        function displayMap2D(obj,ax)

            if nargin >= 2 && ~isempty(ax)
                axes(ax)
            end

            hold on
            axis equal
            grid on
            box on

            xlim([0 obj.size.horiz])
            ylim([0 obj.size.vert])

            title("UAS Simulation")
            xlabel("X (m)")
            ylabel("Y (m)")

        end

    end
end
