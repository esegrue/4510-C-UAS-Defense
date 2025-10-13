clc
clear
close all

map = map(100, 100); % Define map and size (vertical, horizontal)

asset1 = asset([45,35]); % Define asset and location (x, y)
asset2 = asset([50,40]);

sensor1 = sensor([25, 25], 15); % Define sensor(s)
sensor2 = sensor([75, 75], 15); % Define sensor(s)

UAS1 = UAS(15, [20, 100], [60,60], 'Search'); % Define UAS (speed, entrance, target, mode)

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);

sim = simulator(map, AOR, UAS1, [sensor1, sensor2], [asset1, asset2], tps=20, animate=true, nfzs=NFZ1); % Define simulator (map, UASs, Assets, Dt, Animate T/F, NFZs)

s = sim.runSim(); % Runsim