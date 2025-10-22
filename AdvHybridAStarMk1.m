clc
clear
%%map setup
costmap = ones(100,100); % start with everything high cost
costmap(5:95, 5:95) = 0; % carve out navigable region
numObstacles = 100; % number of random obstacles
obsSize = 5; % approximate side length of each obstacle (cells)
mapSize = size(costmap,1)
for i = 1:numObstacles
% pick a random upper-left corner inside the inner safe zone
x = randi([11, mapSize - obsSize - 5]);
y = randi([11, mapSize - obsSize - 5]);
% place a square obstacle block
costmap(y:y+obsSize, x:x+obsSize) = 1;
end
goal = [90, 90, 0]; % x, y, yaw in world coords
imin = 5;
imax = 20;
xStart = randi([imin imax]);
yStart = randi([imin imax]);
start = [imin, imax, 0];
map = binaryOccupancyMap(costmap); %100x100 resolution 2 grid
% figure;
% show(map);
% title('Occupancy Map');
%%
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits;
map.YWorldLimits;
-pi pi];
sv = validatorOccupancyMap(ss, 'Map', map);
sv.ValidationDistance = 0.2; %distance collision sampler
%%planner
minTurningRadius = 5; % meters
planner = plannerHybridAStar(sv, 'MinTurningRadius', minTurningRadius);
%% path
refpath = plan(planner, start, goal);
%%plotted
figure;
show(map); hold on;
pts = refpath.States; % [x y theta]
plot(pts(:,1), pts(:,2), 'r-', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
axis equal;
title('Hybrid A* Path');
legend('Planned Path','Start','Goal', 'location', 'northwest');
hold off;





%%% Next Variables %%%
%Multiple Adversaries (Swarm)
%Reactive adversaries