clear; clc; close all;

map = map(100, 100);                                                        % create map object in map.m:  (L, L)

asset1 = asset([30,60]);                                                    % create asset object(s) in asset.m:  (x, y)
asset2 = asset([60,45]);                                       % use ingressPosns.m function to make entrances = [x1, y1; x2, y2; etc.]

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);                % create NFZ polygon using polyshape fxn: ([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5])

AORo = (asset1.location + asset2.location)/2;
AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);                        % define AOR within greater map using polyshape fxn: ([x1, x2, x3, x4], [y1, y2, y3, y4])

effectorLocations = effectorPosnsGenerator(20, AOR, 2);                     % create matrix of every possible effector location combination using effectorPosnGenerator function:  (effectorResolution, mapSize, numSensors)
effectorRange = 10;
entrances = ingressPosns(10, 100);    

myResults = zeros(length(entrances), 1);                                                  % preallocate results vector (1 = kill, 0 = no kill, [2 = NFZ hit])

CostperCombo = zeros(height(effectorLocations), 1);

for j = 1:height(effectorLocations)
    effector1 = effector([effectorLocations(j, 1), effectorLocations(j, 2)], effectorRange);   % create effector object(s) in effector.m:  ([x, y], range)
    effector2 = effector([effectorLocations(j, 3), effectorLocations(j, 4)], effectorRange);
    parfor i = 1:height(entrances)                                             % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [effector1, effector2], [asset1], tps=20, animate=false, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
        myResults(i) = sim.runSim.cost();
    end
    CostperCombo(j) = sum(myResults, 'all')/length(myResults);
end

% Identify effector location combination with highest DP

minCostID = find(min(CostperCombo) == CostperCombo);


for j = 1:length(minCostID)
    ID = minCostID(j);
    effector1 = effector([effectorLocations(ID, 1), effectorLocations(ID, 2)], effectorRange);    % manually create, simulate, and animate optimal effector placement
    effector2 = effector([effectorLocations(ID, 3), effectorLocations(ID, 4)], effectorRange);
    parfor i = 1:height(entrances)                                                 % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
            sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [effector1, effector2], [asset1], tps=20, animate=true, nfzs=NFZ1, resetGraphics=true, animationMultiplier=100);
            myResults(i) = sim.runSim.UASkilled();
    end
end