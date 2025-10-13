clc
clear
close all

map = map(100, 100);                                                        % create map object in map.m:  (L, L)

asset1 = asset([30,60]);                                                    % create asset object(s) in asset.m:  (x, y)
asset2 = asset([60,45]);

sensorLocations = sensorPosnsGenerator(20, 100, 2);                         % create matrix of every possible sensor location combination using sensorPosnGenerator function:  (sensorResolution, mapSize, numSensors)
sensorRange = 10;
entrances = ingressPosns(100, 100);                                         % use ingressPosns.m function to make entrances = [x1, y1; x2, y2; etc.]

%UAS1 = UAS(15, [80, 0], asset1.location, 'Linear');                        % example of creating UAS object in UAS.m:   (speed, entrance, target, mode)

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);                % create NFZ polygon using polyshape fxn: ([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5])

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);                        % define AOR within greater map using polyshape fxn: ([x1, x2, x3, x4], [y1, y2, y3, y4])

%sim = simulator(map, AOR, UAS1, [sensor1], [asset1], 0.05, 0, NFZ1);       % example of creating sim object in simulator.m:  (Map, AOR, UASs, Sensors, Assets, tps=20, aniamte=true, nfzs=[], resetGraphics=true, animationMultiplier=100)

myResults = zeros(100, 1);                                                  % preallocate results vector (1 = kill, 0 = no kill, [2 = NFZ hit])

DPperCombo = zeros(height(sensorLocations), 1);

for j = 1:height(sensorLocations)
    sensor1 = sensor([sensorLocations(j, 1), sensorLocations(j, 2)], sensorRange);   % create sensor object(s) in sensor.m:  ([x, y], range)
    sensor2 = sensor([sensorLocations(j, 3), sensorLocations(j, 4)], sensorRange);
    parfor i = 1:height(entrances)                                             % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [sensor1, sensor2], [asset1], tps=20, animate=false, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
        myResults(i) = sim.runSim.UASSensed();
    end
    DPperCombo(j) = sum(myResults, 'all')/length(myResults);
end

% Identify sensor location combination with highest DP

[~, idx] = max(DPperCombo);
maxDProw = idx;                                                             % row index in DPperCombo with highest DP

sensor1 = sensor([sensorLocations(idx, 1), sensorLocations(idx, 2)], sensorRange);    % manually create, simulate, and animate optimal sensor placement
sensor2 = sensor([sensorLocations(idx, 3), sensorLocations(idx, 4)], sensorRange);

for i = 1:height(entrances)                                                 % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [sensor1, sensor2], [asset1], tps=20, animate=true, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
        myResults(i) = sim.runSim.UASSensed();
end


% Bugs to address
    % generate one matrix (struct) with sensor objects which line 30 can reference
    % directly instead of creating sensor objects (line 27, 28) for each iteration of
    % sensor combinations

    % have the drones attack average location of each sensor and run on
    % 'search' mode

    % implement sensor standoff distance from assets so the highest DP combo isn't
    % simply the combo which has sensors directly on top of assets (and
    % address why this isnt the case for this specific scenario?!) 

    % fix detection probability (include NFZ hits?) etc. to get correct
    % detection probability for Dr. Kumar