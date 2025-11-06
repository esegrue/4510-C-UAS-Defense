clear; clc; close all;

map = map(100, 100);                                                        % create map object in map.m:  (L, L)

asset1 = asset([30,60]);                                                    % create asset object(s) in asset.m:  (x, y)
asset2 = asset([60,45]);                                       % use ingressPosns.m function to make entrances = [x1, y1; x2, y2; etc.]

sensorRange = 10;
sensor1 = sensors([30, 60], sensorRange); 
sensor2 = sensors([60, 45], sensorRange);

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);                % create NFZ polygon using polyshape fxn: ([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5])

AORo = (asset1.location + asset2.location)/2;
AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);                        % define AOR within greater map using polyshape fxn: ([x1, x2, x3, x4], [y1, y2, y3, y4])

all_effectorLocations = effectorPosnsGenerator(20, AOR, 2);                     % create matrix of every possible effector location combination using effectorPosnGenerator function:  (effectorResolution, mapSize, numSensors)
tot_effectorLocations = length(all_effectorLocations);
effectorRange = 10;
all_entrances = ingressPosns(100, 100);
tot_entrances = length(all_entrances);

max_runs = 100;
num_tests = 10;
entrances = all_entrances(randi(tot_entrances, num_tests, 1),:);

myResults = nan(num_tests, 1);                                                  % preallocate results vector (1 = kill, 0 = no kill, [2 = NFZ hit])
CostperCombo = nan(tot_effectorLocations, 1);
LCB = nan(tot_effectorLocations);
UCB = nan(tot_effectorLocations);

runs = 0;
maxRuns = tot_effectorLocations;
delta = 10;
while runs <= maxRuns
    runs = runs + 1;
    i = runs;
    effectorLocations(i,:) = all_effectorLocations(randi(tot_effectorLocations, 1),:);
    entrances = all_entrances(randi(tot_entrances, num_tests, 1), :);
    effector1 = effector([effectorLocations(i, 1), effectorLocations(i, 2)], effectorRange);   % create effector object(s) in effector.m:  ([x, y], range)
    effector2 = effector([effectorLocations(i, 3), effectorLocations(i, 4)], effectorRange);
    parfor j = 1:num_tests                                         % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        sim = simulator(map, AOR, UAS(15, entrances(j, :), asset1.location, 'Linear'), [effector1, effector2], [sensor1, sensor2], [asset1], tps=20, animate=false, nfzs=NFZ1, resetGraphics=true, animationMultiplier=100);
        myResults(j) = sim.runSim.cost();
    end
    CostperCombo(i) = sum(myResults, 'all')/num_tests; % estimated cost
    SD(i) = std(myResults, 1); % sample standard deviation
    SE(i) = SD(i)/sqrt(num_tests); % standard error of the mean

    % confidence interval
    alpha = 0.1; % 90 percent confidence
    tcrit = tinv(1 - alpha/2, num_tests - 1); % Student's t-distribution
    LCB(i) = CostperCombo(i) - tcrit*SE(i); % lower confidence bound
    UCB(i) = CostperCombo(i) + tcrit*SE(i); % upper confidence bound

    runsMask = find(~isnan(CostperCombo)); % indices of completed runs
    [~, b] = min(CostperCombo(runsMask)); % best config
    rivals = setdiff(runsMask, b); % all configs except best
    if isempty(rivals)
        separation(i) = -inf;
    else
        separation(i) = min(LCB(rivals)) - UCB(b); 
    end
    if separation(i) > delta
        break
    end
end
minCostID = b;
for j = 1%:length(minCostID)
    ID = minCostID(j);
    effector1 = effector([effectorLocations(ID, 1), effectorLocations(ID, 2)], effectorRange);    % manually create, simulate, and animate optimal effector placement
    effector2 = effector([effectorLocations(ID, 3), effectorLocations(ID, 4)], effectorRange);
    parfor i = 1:height(entrances)                                                 % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
            sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [effector1, effector2], [sensor1, sensor2], [asset1], tps=20, animate=true, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
            myResults(i) = sim.runSim.UASkilled();
    end
end