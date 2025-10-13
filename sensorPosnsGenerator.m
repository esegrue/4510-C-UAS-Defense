% Generates a matrix with every possible combination of sensor locations

    % Input: 
        % sensorResolution - resolution (distance between each sensor)
        % mapSize - length of one size of the map
        % numSensors - number of sensors on the map

    % Output: 
        % sensorLocations = [x1, y1, x2, y2, x3, y3; -> combination 1
        %                    x1, y1, x2, y2, x3, y3; -> combination 2
        %                    x1, y1, x2, y2, x3, y3; -> combination 3
        %                    ... ... ... ... ... ... -> combination n

        % sensorInputs = [(x, y
            

function sensorLocations = sensorPosnsGenerator(sensorResolution, mapSize, numSensors)

    coords = 0:sensorResolution:mapSize;                                    % generate all possible grid coordinates
    [X, Y] = meshgrid(coords, coords);
    points = [X(:), Y(:)];                                                  % (x,y) pairs
    numPoints = size(points,1);

    combos = nchoosek(1:numPoints, numSensors);                             % generate all sensor location combonations

    
    sensorLocations = zeros(size(combos,1), 2*numSensors);                  % preallocate sensorLocations matrix

    
    for i = 1:size(combos,1)                                                % build rows: [x1, y1, x2, y2, ...]
        chosenPoints = points(combos(i,:), :);  
        sensorLocations(i,:) = reshape(chosenPoints.', 1, []);  
    end
end

  