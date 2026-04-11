clear; clc; close all;

mobilityType = "Mobile";
weaponType = "Mixed";

baseDir = fullfile(pwd, 'Simulation_Results');

fprintf('Searching for scenario summary files in: %s\n', baseDir);
filePattern = fullfile(baseDir, '**', '*_Summary_Latest.mat');
summaryFiles = dir(filePattern);

numFiles = length(summaryFiles);
if numFiles == 0
    error('No summary files found. Check your directory path.');
end
fprintf('Found %d scenario summaries. Compiling data...\n\n', numFiles);

AdvCount = zeros(numFiles, 1);
EffCount = zeros(numFiles, 1);
Mobility = strings(numFiles, 1);
Weapon = strings(numFiles, 1);
BestCost = nan(numFiles, 1);
BestReliability = nan(numFiles, 1);
IterationsRun = zeros(numFiles, 1);
BestLCB = nan(numFiles, 1);
BestUCB = nan(numFiles, 1);
Uncertainty = nan(numFiles, 1);

for i = 1:numFiles
    filePath = fullfile(summaryFiles(i).folder, summaryFiles(i).name);
    
    data = load(filePath, 'SummaryData');
    S = data.SummaryData;
    
    % Expected format: Adv{d}_Eff{d}_{str}_{str}_Summary_Latest.mat
    tokens = regexp(summaryFiles(i).name, 'Adv(\d+)_Eff(\d+)_([A-Za-z]+)_([A-Za-z]+)_Summary_Latest', 'tokens');
    
    if ~isempty(tokens)
        AdvCount(i) = str2double(tokens{1}{1});
        EffCount(i) = str2double(tokens{1}{2});
        Mobility(i) = string(tokens{1}{3});
        Weapon(i) = string(tokens{1}{4});
    else
        warning('Filename %s did not match expected naming convention.', summaryFiles(i).name);
    end
    
    IterationsRun(i) = S.NumConfigsRun;
    bestID = S.BestID;

    if bestID > 0 && bestID <= length(S.CostperCombo)
        BestCost(i) = S.CostperCombo(bestID);
        BestReliability(i) = S.ReliabilityScore(bestID);
        BestLCB(i) = S.LCB(bestID); 
        BestUCB(i) = S.UCB(bestID);
        Uncertainty(i) = BestUCB(i) - BestLCB(i);
    else
        BestCost(i) = NaN; 
        BestReliability(i) = NaN;
        BestLCB(i) = NaN;   
        BestUCB(i) = NaN;   
        Uncertainty(i) = NaN;
    end
end

ScenarioResults = table(AdvCount, EffCount, Mobility, Weapon, BestCost, BestReliability, BestLCB, BestUCB, Uncertainty, IterationsRun);

ScenarioResults = sortrows(ScenarioResults, {'AdvCount', 'EffCount'});
disp(ScenarioResults);

%% Top Performers
% ---------------

[topCost, topCostID] = min(ScenarioResults.BestCost);
topCostPerformer = ScenarioResults(topCostID, :);

[topReliability, topReliabilityID] = max(ScenarioResults.BestReliability);
topReliabilityPerformer = ScenarioResults(topReliabilityID, :);

[topRisk, topRiskID] = min(ScenarioResults.Uncertainty);
topRiskPerformer = ScenarioResults(topRiskID, :);


%% OPTIMAL COST ACROSS EFFECTOR COUNT
% -----------------------------------

figure('Name', 'Cost Comparison Across Scenarios', 'Color', 'w');
try
    subset = ScenarioResults(ScenarioResults.Mobility == mobilityType & ScenarioResults.Weapon == weaponType, :);
    
    if ~isempty(subset)
        plotData = unstack(subset(:, {'AdvCount', 'EffCount', 'BestCost'}), 'BestCost', 'AdvCount');
        
        % Plotting
        xVals = plotData.EffCount; % Now setting x-axis to Effector Count
        yVals = table2array(plotData(:, 2:end)); 
        
        b = bar(xVals, yVals);
        title({'Optimal Defensive Cost vs. Number of Effectors', '(' + mobilityType + ' Mobility & ' + weaponType + ' Weapons)'}, 'FontSize', 14);
        xlabel('Number of Effectors', 'FontSize', 12);
        ylabel('Cost of Best Configuration ($)', 'FontSize', 12);
        grid on;
        
        advLabels = plotData.Properties.VariableNames(2:end);
        advLabels = strrep(advLabels, 'x', 'Adversaries: '); % Clean up variable names
        legend(advLabels, 'Location', 'bestoutside');
    end
catch
    disp('Not enough varied data to generate the comparative bar plot yet.');
end

%% OPTIMAL RELIABILITY ACROSS EFFECTOR COUNT
% -----------------------------------

figure('Name', 'Reliability Comparison Across Scenarios', 'Color', 'w');
try
    subset = ScenarioResults(ScenarioResults.Mobility == mobilityType & ScenarioResults.Weapon == weaponType, :);
    
    if ~isempty(subset)
        plotData = unstack(subset(:, {'AdvCount', 'EffCount', 'BestReliability'}), 'BestReliability', 'AdvCount');
        
        % Plotting
        xVals = plotData.EffCount; 
        yVals = table2array(plotData(:, 2:end)); 
        
        b = bar(xVals, yVals);
        title({'Optimal Defensive Reliability vs. Number of Effectors', '(' + mobilityType + ' Mobility & ' + weaponType + ' Weapons)'}, 'FontSize', 14);
        xlabel('Number of Effectors', 'FontSize', 12);
        ylabel('Reliability of Best Configuration (%)', 'FontSize', 12);
        grid on;
        
        advLabels = plotData.Properties.VariableNames(2:end);
        advLabels = strrep(advLabels, 'x', 'Adversaries: '); % Clean up variable names
        legend(advLabels, 'Location', 'bestoutside');
    end
catch
    disp('Not enough varied data to generate the comparative bar plot yet.');
end

%% TRADE-OFF ANALYSIS: COST VS RELIABILITY SCATTER
% ------------------------------------------------

figure('Name', 'Trade-off: Cost vs Reliability', 'Color', 'w', 'Position', [100 100 900 600]);
try
    hold on;
    
    uniqueMobility = unique(ScenarioResults.Mobility);
    uniqueWeapon = unique(ScenarioResults.Weapon);
    
    colorMap = lines(length(uniqueMobility));
    markerShapes = {'o', 's', '^', 'd'}; % Circle, Square, Triangle, Diamond
    
    for i = 1:height(ScenarioResults)
        if isnan(ScenarioResults.BestCost(i)), continue; end
        
        mIdx = find(uniqueMobility == ScenarioResults.Mobility(i));
        wIdx = find(uniqueWeapon == ScenarioResults.Weapon(i));
        
        scatter(ScenarioResults.BestCost(i), ScenarioResults.BestReliability(i), ...
            150, colorMap(mIdx, :), markerShapes{wIdx}, 'filled', 'MarkerEdgeColor', 'k', ...
            'HandleVisibility', 'off'); 
        
        % Annotation
        text(ScenarioResults.BestCost(i), ScenarioResults.BestReliability(i), sprintf('    %dE vs %dA', ScenarioResults.EffCount(i), ScenarioResults.AdvCount(i)), 'FontSize', 9, 'HandleVisibility', 'off', 'FontWeight', 'bold');
    end
    
    title('Master Trade-off: Cost vs. Reliability', 'FontSize', 14);
    xlabel('Best Configuration Cost ($)', 'FontSize', 12);
    ylabel('System Reliability (%)', 'FontSize', 12);
    grid on;
    
    dummyPlots = []; legendLabels = {};
    
    for m = 1:length(uniqueMobility)
        h = plot(nan, nan, 'o', 'MarkerFaceColor', colorMap(m, :), 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Mobility: %s', uniqueMobility(m));
    end
    
    for w = 1:length(uniqueWeapon)
        h = plot(nan, nan, markerShapes{w}, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Weapon: %s', uniqueWeapon(w));
    end
    
    lgd = legend(dummyPlots, legendLabels, 'Location', 'bestoutside', 'NumColumns', 1);
    lgd.AutoUpdate = 'off';
    hold off;
    
catch ME
    disp('Could not generate Master Trade-off plot.');
    disp(ME.message);
end

%% RISK ANALYSIS (UCB - LCB)
% --------------------------

figure('Name', 'Master Volatility Scatter', 'Color', 'w', 'Position', [150 150 900 600]);
try
    hold on;
    ScenarioResults.Uncertainty = ScenarioResults.BestUCB - ScenarioResults.BestLCB;
    
    uniqueMobility = unique(ScenarioResults.Mobility);
    uniqueWeapon = unique(ScenarioResults.Weapon);
    
    colorMap = lines(length(uniqueMobility));
    markerShapes = {'o', 's', '^', 'd'}; % Circle, Square, Triangle, Diamond
    
    for i = 1:height(ScenarioResults)
        if isnan(ScenarioResults.Uncertainty(i)), continue; end
        
        mIdx = find(uniqueMobility == ScenarioResults.Mobility(i));
        wIdx = find(uniqueWeapon == ScenarioResults.Weapon(i));
        
        scatter(ScenarioResults.BestCost(i), ScenarioResults.Uncertainty(i), ...
            150, colorMap(mIdx, :), markerShapes{wIdx}, 'filled', 'MarkerEdgeColor', 'k', ...
            'HandleVisibility', 'off'); 
        
        % Annotation
        text(ScenarioResults.BestCost(i), ScenarioResults.Uncertainty(i), ...
            sprintf('    %dE vs %dA', ScenarioResults.AdvCount(i), ScenarioResults.EffCount(i)), ...
            'FontSize', 9, 'HandleVisibility', 'off', 'FontWeight', 'bold');
    end
    
    title('System Risk vs Cost', 'FontSize', 14);
    xlabel('Best Configuration Cost ($)', 'FontSize', 12);
    ylabel('Outcome Uncertainty (UCB - LCB)', 'FontSize', 12);
    grid on;
    
    dummyPlots = []; legendLabels = {};
    
    for m = 1:length(uniqueMobility)
        h = plot(nan, nan, 'o', 'MarkerFaceColor', colorMap(m, :), 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Mobility: %s', uniqueMobility(m));
    end
    
    for w = 1:length(uniqueWeapon)
        h = plot(nan, nan, markerShapes{w}, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Weapon: %s', uniqueWeapon(w));
    end
    
    lgd2 = legend(dummyPlots, legendLabels, 'Location', 'bestoutside', 'NumColumns', 1);
    lgd2.AutoUpdate = 'off';
    hold off;
    
catch ME
    disp('Could not generate Master Volatility plot.');
    disp(ME.message);
end

%% PERFORMANCE HEATMAP
% --------------------

figure('Name', 'Reliability Heatmap', 'Color', 'w');
try
    subset = ScenarioResults(ScenarioResults.Mobility == mobilityType & ScenarioResults.Weapon == weaponType, :);
    
    if ~isempty(subset)
        % Create a 2D grid for the heatmap
        uniqueAdv = unique(subset.AdvCount);
        uniqueEff = unique(subset.EffCount);
        heatMatrix = nan(length(uniqueAdv), length(uniqueEff));
        
        for a = 1:length(uniqueAdv)
            for e = 1:length(uniqueEff)
                idx = find(subset.AdvCount == uniqueAdv(a) & subset.EffCount == uniqueEff(e));
                if ~isempty(idx)
                    heatMatrix(a, e) = subset.BestReliability(idx);
                end
            end
        end
        
        h = heatmap(uniqueEff, uniqueAdv, heatMatrix);
        h.Title = {'Reliability Heatmap', '(' + mobilityType + ' Mobility & ' + weaponType + ' Weapons)'};
        h.XLabel = 'Number of Effectors';
        h.YLabel = 'Number of Adversaries';
        h.Colormap = parula; % Standard color scale
        h.ColorLimits = [0 100]; % Reliability goes from 0 to 100%
        
        h.CellLabelFormat = '%.1f%%';
    end
catch
    disp('Could not generate Reliability Heatmap.');
end

%% AVERAGED PERFORMANCE: COST VS RELIABILITY (AVERAGED ACROSS ADVERSARIES)
% ------------------------------------------------------------------------

figure('Name', 'Averaged Trade-off: Cost vs Reliability', 'Color', 'w', 'Position', [200 200 900 600]);
try
    % Group by Effector, Mobility, and Weapon to average across Adversaries
    avgResults = groupsummary(ScenarioResults, {'EffCount', 'Mobility', 'Weapon'}, 'mean', {'BestCost', 'BestReliability'});
    
    hold on;
    
    uniqueMobility = unique(avgResults.Mobility);
    uniqueWeapon = unique(avgResults.Weapon);
    
    colorMap = lines(length(uniqueMobility));
    markerShapes = {'o', 's', '^', 'd'}; % Circle, Square, Triangle, Diamond
    
    for i = 1:height(avgResults)
        if isnan(avgResults.mean_BestCost(i)) || isnan(avgResults.mean_BestReliability(i)), continue; end
        
        mIdx = find(uniqueMobility == avgResults.Mobility(i));
        wIdx = find(uniqueWeapon == avgResults.Weapon(i));
        
        scatter(avgResults.mean_BestCost(i), avgResults.mean_BestReliability(i), ...
            150, colorMap(mIdx, :), markerShapes{wIdx}, 'filled', 'MarkerEdgeColor', 'k', ...
            'HandleVisibility', 'off'); 
        
        text(avgResults.mean_BestCost(i), avgResults.mean_BestReliability(i), ...
            sprintf('    %dE', avgResults.EffCount(i)), 'FontSize', 9, 'HandleVisibility', 'off', 'FontWeight', 'bold');
    end
    
    title('Averaged Performance: Cost vs. Reliability', 'FontSize', 14);
    subtitle('Averaged across Adversary Swarm Sizes', 'FontSize', 11);
    xlabel('Average Best Configuration Cost ($)', 'FontSize', 12);
    ylabel('Average System Reliability (%)', 'FontSize', 12);
    grid on;
    
    dummyPlots = []; legendLabels = {};
    
    for m = 1:length(uniqueMobility)
        h = plot(nan, nan, 'o', 'MarkerFaceColor', colorMap(m, :), 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Mobility: %s', uniqueMobility(m));
    end
    
    for w = 1:length(uniqueWeapon)
        h = plot(nan, nan, markerShapes{w}, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, 'LineStyle', 'none');
        dummyPlots = [dummyPlots, h]; 
        legendLabels{end+1} = sprintf('Weapon: %s', uniqueWeapon(w));
    end
    
    lgd3 = legend(dummyPlots, legendLabels, 'Location', 'bestoutside', 'NumColumns', 1);
    lgd3.AutoUpdate = 'off';
    hold off;
    
catch ME
    disp('Could not generate Averaged Trade-off plot.');
    disp(ME.message);
end