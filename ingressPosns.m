function [pos] = ingressPosns(xlim, ylim)
    % Generates a random [x,y] coordinate from a range of x and y values
    % along the perimeter

    % Inputs -
    % xlim: range of x values [x0,xend]
    % ylim: range of y values [y0,yend]
    % Outputs -
    % pos: position along perimeter [x,y]
    
    x_min = xlim(1);
    x_max = xlim(2);
    y_min = ylim(1);
    y_max = ylim(2);

    width = x_max - x_min;
    height = y_max - y_min;
    
    % Total length of the perimeter
    perimeter_len = 2 * width + 2 * height;

    % Generate random distances along the unfolded perimeter (0 to perimeter_len)
    d = rand() * perimeter_len;

    % Initialize output matrix
    pos = nan([1 2]);

    % --- Map the 1D distance 'd' back to 2D coordinates ---

    % 1. Bottom Edge (x changes, y is min)
    % Range: [0, width)
    idx = d < width;
    pos(idx, 1) = x_min + d(idx);
    pos(idx, 2) = y_min;

    % 2. Right Edge (x is max, y changes)
    % Range: [width, width + height)
    idx = (d >= width) & (d < (width + height));
    pos(idx, 1) = x_max;
    pos(idx, 2) = y_min + (d(idx) - width);

    % 3. Top Edge (x changes, y is max)
    % Range: [width + height, 2*width + height)
    idx = (d >= (width + height)) & (d < (2*width + height));
    % Note: Subtracting to go "backwards" (right to left) keeps visual continuity,
    % but mathematically random left-to-right is fine too. Here we go right to left
    pos(idx, 1) = x_max - (d(idx) - (width + height));
    pos(idx, 2) = y_max;

    % 4. Left Edge (x is min, y changes)
    % Range: [2*width + height, Total)
    idx = d >= (2*width + height);
    pos(idx, 1) = x_min;
    pos(idx, 2) = y_max - (d(idx) - (2*width + height));
end