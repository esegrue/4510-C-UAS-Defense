% Generates a matrix with every possible combination of effector locations

    % Input: 
        % effectorResolution - resolution (distance between each effector)
        % mapSize - length of one size of the map
        % numEffectors - number of effectors on the map

    % Output: 
        % effectorLocations = [x1, y1, x2, y2, x3, y3; -> combination 1
        %                    x1, y1, x2, y2, x3, y3; -> combination 2
        %                    x1, y1, x2, y2, x3, y3; -> combination 3
        %                    ... ... ... ... ... ... -> combination n

        % effectorInputs = [(x, y
            

function effectorLocations = effectorPosnsGenerator(effectorResolution, AOR, numEffectors)
    
    Xcoords = min(AOR.Vertices(:,1)):effectorResolution:max(AOR.Vertices(:,1));
    Ycoords = min(AOR.Vertices(:,2)):effectorResolution:max(AOR.Vertices(:,2));
    %Xcoords = linspace(min(AOR.Vertices(:,1)), max(AOR.Vertices(:,1)), effectorResolution);
    %Ycoords = linspace(min(AOR.Vertices(:,2)), max(AOR.Vertices(:,2)), effectorResolution);
    [X, Y] = meshgrid(Xcoords, Ycoords);
    points = [X(:), Y(:)];                                                  % (x,y) pairs
    numPoints = size(points,1);

    combos = nchoosek(1:numPoints, numEffectors);                             % generate all effector location combonations

    
    effectorLocations = zeros(size(combos,1), 2*numEffectors);                  % preallocate effectorLocations matrix

    
    for i = 1:size(combos,1)                                                % build rows: [x1, y1, x2, y2, ...]
        chosenPoints = points(combos(i,:), :);  
        effectorLocations(i,:) = reshape(chosenPoints.', 1, []);  
    end
end

  