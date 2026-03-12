classdef simulator < handle

    properties
        map
        AOR
        UAS
        UASPos_all
        effectors
        sensors
        assets

        defenders
        obstacles
        NFZunion

        tick
        dt
        tps
        animate
        NFZs
        resetGraphics
        animationMultiplier
        hideClock
        fadePings
        costConfig
        effectors3D
        occMap

        grid2D
        showFigure
        maxSimTime
        drawEvery

        stopRule

        weaponMode
        kineticProjectileSpeed
        kineticShotsPerVolley
        kineticHitProbability
        kineticRefireTime
        projectileHitTolerance

        effectorLastFireTime
        projectiles
    end


    methods

        function obj = simulator(map,aor,uas,effectors,sensors,assets,options)

            arguments
                map
                aor
                uas
                effectors
                sensors
                assets

                options.tps = 20
                options.animate = true
                options.nfzs = polyshape.empty
                options.defenders = []
                options.obstacles = polyshape.empty

                options.weaponMode = "KINETIC"
                options.kineticProjectileSpeed = 30
                options.kineticShotsPerVolley = 1
                options.kineticHitProbability = 0.7
                options.kineticRefireTime = 0.5
                options.projectileHitTolerance = 0.75
            end

            obj.map = map;
            obj.AOR = aor;
            obj.UAS = uas;
            obj.effectors = effectors;
            obj.sensors = sensors;
            obj.assets = assets;

            obj.defenders = options.defenders;
            obj.obstacles = options.obstacles;

            obj.tps = options.tps;
            obj.dt = 1/obj.tps;

            obj.animate = options.animate;
            obj.NFZs = options.nfzs;

            obj.weaponMode = upper(options.weaponMode);

            obj.kineticProjectileSpeed = options.kineticProjectileSpeed;
            obj.kineticShotsPerVolley = options.kineticShotsPerVolley;
            obj.kineticHitProbability = options.kineticHitProbability;
            obj.kineticRefireTime = options.kineticRefireTime;
            obj.projectileHitTolerance = options.projectileHitTolerance;

            obj.projectiles = [];
            obj.effectorLastFireTime = zeros(length(effectors),1);

            obj.occMap = obj.map.getOccMap(28);

            obj.UASPos_all = cell(1,length(obj.UAS));

            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
            end

        end


        function results = runSim(obj)

            numUAS = length(obj.UAS);

            uasActive = true(numUAS,1);

            tickCount = 0;

            while any(uasActive)

                tickCount = tickCount + 1;
                obj.tick = tickCount;

                for i = 1:numUAS

                    if ~uasActive(i)
                        continue
                    end

                    uasObj = obj.UAS(i);

                    modeStr = upper(string(uasObj.mode));

                    switch modeStr

                        case "LINEAR"
                            uasObj.linearMotion(obj.dt);

                        case "HYBRIDASTAR"
                            uasObj.hybridAStarMotion(obj.dt,tickCount,3.0,obj.occMap);

                        case "SEARCH"
                            uasObj.searchMotion(obj.dt,obj.assets,[],obj.NFZs);

                    end

                    obj.UAS(i) = uasObj;

                    pos = uasObj.position;

                    obj.UASPos_all{i} = cat(1,obj.UASPos_all{i},pos);

                end

                if obj.weaponMode == "KINETIC"
                    [obj,uasActive] = obj.stepKinetic(uasActive);
                end

            end

            results.UASPos_all = obj.UASPos_all;

        end


        function [obj,uasActive] = stepKinetic(obj,uasActive)

            for e = 1:length(obj.effectors)

                eff = obj.effectors(e);

                for i = 1:length(obj.UAS)

                    if ~uasActive(i)
                        continue
                    end

                    uasPos = obj.UAS(i).position(1:2);

                    d = norm(uasPos - eff.location);

                    if d > eff.range
                        continue
                    end

                    if rand <= obj.kineticHitProbability

                        uasActive(i) = false;

                        pos = obj.UAS(i).position;

                        obj.map.animateUASkilled(pos);

                    end

                end

            end

        end

    end

end
