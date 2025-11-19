function [pos] = effectorPosnsGenerator(xlim, ylim)
    % Generates a random [x,y] coordinate from a range of x and y values

    % Inputs -
    % xlim: range of x values [x1, xend]
    % ylim: range of y values [y1, yend]
    % Outputs -
    % pos: position [x, y]
    
    x0 = xlim(1);
    y0 = ylim(1);
    xdiff = xlim(2) - x0;
    ydiff = ylim(2) - y0;
    dx = rand()*xdiff;
    dy = rand()*ydiff;
    pos = [x0 + dx, y0 + dy];
end