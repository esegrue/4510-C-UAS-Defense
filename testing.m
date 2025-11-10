clc, clear, close all


costmap = zeros(100,100);
costmap(5:end-5, 5:end-5) = 1;

obssize = 1;
mapsize = size(costmap, 1);
costmap(rand(mapsize) > 0.25) = 0

goal = [50,50,0];
costmap(goal(1), goal(2)) = 0;

map = binaryOccupancyMap(costmap);


ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; -pi pi];


sv = validatorOccupancyMap(ss, 'Map', map);
start = [0,0,0];
minTurningRadius =5;
planner = plannerHybridAStar(sv, 'MinTurningRadius',minTurningRadius);
refpath = plan(planner, start, goal)


figure()
show(map)
hold on
pts = refpath.States;
plot(pts(:,1), pts(:,2), 'r-')