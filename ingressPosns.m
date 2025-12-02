function [pos] = ingressPosns(xlim, ylim, numUAS)
    % Randomly generates positions of all UAS ingressions along the map
    % perimeter

    % Inputs:
    % xlim: [a,b] width of map
    % ylim: [c,d] height of map
    % numUAS: umber of UAS in a scenario (N)

    % Output:
    % pos: [N x 2] (x1,y1; x2,y2;...;xN,yN)
    
    arguments
        xlim (1,2) double
        ylim (1,2) double
        numUAS (1,1) double = 1 % default 1 adversary
    end
    
    x_min = xlim(1); x_max = xlim(2);
    y_min = ylim(1); y_max = ylim(2);

    width = x_max - x_min;
    height = y_max - y_min;
    perimeter_len = 2 * width + 2 * height;

    % Generate N random distances along the perimeter
    d = rand(numUAS, 1) * perimeter_len;
    pos = nan(numUAS, 2);

    % Vectorized Mapping
    % 1. Bottom Edge
    idx = d < width;
    pos(idx, 1) = x_min + d(idx);
    pos(idx, 2) = y_min;

    % 2. Right Edge
    idx = (d >= width) & (d < (width + height));
    pos(idx, 1) = x_max;
    pos(idx, 2) = y_min + (d(idx) - width);

    % 3. Top Edge
    idx = (d >= (width + height)) & (d < (2*width + height));
    pos(idx, 1) = x_max - (d(idx) - (width + height));
    pos(idx, 2) = y_max;

    % 4. Left Edge
    idx = d >= (2*width + height);
    pos(idx, 1) = x_min;
    pos(idx, 2) = y_max - (d(idx) - (2*width + height));
end