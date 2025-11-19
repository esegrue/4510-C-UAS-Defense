clear; clc; close all;

map = map(100, 100);                                                        % create map object in map.m:  (L, L)
xlim = [0 100]; ylim = [0 100];

asset1 = asset([30,60]);                                                    % create asset object(s) in asset.m:  (x, y)
asset2 = asset([60,45]);                                       % use ingressPosns.m function to make entrances = [x1, y1; x2, y2; etc.]

sensorRange = 10;
sensor1 = sensors([30, 60], sensorRange); 
sensor2 = sensors([60, 45], sensorRange);

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);                % create NFZ polygon using polyshape fxn: ([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5])

AORo = (asset1.location + asset2.location)/2;
AORsize = 60;
AOR = polyshape([AORo(1)-AORsize/2, AORo(1)-AORsize/2, AORo(1)+AORsize/2, AORo(1)+AORsize/2], [AORo(2)+AORsize/2, AORo(2)-AORsize/2, AORo(2)-AORsize/2, AORo(2)+AORsize/2]);                        % define AOR within greater map using polyshape fxn: ([x1, x2, x3, x4], [y1, y2, y3, y4])

effectorRange = 10;

max_configs = 100;
num_tests = 10;
myResults = nan(num_tests, 1);                                                  % preallocate results vector (1 = kill, 0 = no kill, [2 = NFZ hit])
CostperCombo = nan(max_configs, 1);
LCB = nan(max_configs);
UCB = nan(max_configs);

num_configs = 0;
delta = 10;
while num_configs < max_configs
    num_configs = num_configs + 1;
    i = num_configs;
    effectorPos1 = effectorPosnsGenerator(xlim, ylim);
    effectorPos2 = effectorPosnsGenerator(xlim, ylim);
    effectorPos(i,:) = [effectorPos1 effectorPos2];
    effector1 = effector([effectorPos(i,1) effectorPos(i,2)], effectorRange);   % create effector object(s) in effector.m:  ([x, y], range)
    effector2 = effector([effectorPos(i,3) effectorPos(i,4)], effectorRange);
    parfor j = 1:num_tests % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        adversaryPos1 = ingressPosns(xlim, ylim);
        adversaryPos(j,:) = [adversaryPos1];
        uas = UAS(15, adversaryPos1, asset1.location, 'Linear');
        sim = simulator(map, AOR, uas, [effector1, effector2], [sensor1, sensor2], [asset1], tps=20, animate=false, nfzs=NFZ1, resetGraphics=true, animationMultiplier=100);
        myResults(j) = sim.runSim.cost();
    end
    CostperCombo(i) = sum(myResults, 'all')/num_tests; % estimated cost
    SD(i) = std(myResults, 1); % sample standard deviation
    SE(i) = SD(i)/sqrt(num_tests); % standard error of the mean

    % confidence interval
    alpha = 0.05; % 90 percent confidence
    tcrit = tinv(1 - alpha/2, num_tests - 1); % Student's t-distribution
    LCB(i) = CostperCombo(i) - tcrit*SE(i); % lower confidence bound
    UCB(i) = CostperCombo(i) + tcrit*SE(i); % upper confidence bound

    runsMask = find(~isnan(CostperCombo)); % indices of completed configs
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
    parfor i = 1:height(adversaryPos)                                                 % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
        uas = UAS(15, adversaryPos(i, :), asset1.location, 'Linear')
        sim = simulator(map, AOR, uas, [effector1, effector2], [sensor1, sensor2], [asset1], tps=20, animate=true, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
        myResults(i) = sim.runSim.UASkilled();
    end
end