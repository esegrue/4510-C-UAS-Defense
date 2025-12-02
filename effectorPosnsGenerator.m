function [pos] = effectorPosnsGenerator(xlim, ylim, numEffectors)
    % Randomly generates positions of all deployed effectors within the
    % deployable area

    % Inputs -
    % xlim: [a,b] width of deployable area
    % ylim: [c,d] height of deployable area
    % numEffectors: number of effectors deployed (N)

    % Outputs -
    % pos: [N x 2] (x1,y1; x2,y2;...;xN,yN)
    
    arguments
        xlim (1,2) double
        ylim (1,2) double
        numEffectors (1,1) double = 1 % default 1 effector
    end

    x0 = xlim(1);
    y0 = ylim(1);
    xdiff = xlim(2) - x0;
    ydiff = ylim(2) - y0;
    
    % Vectorized generation
    dx = rand(numEffectors, 1) * xdiff;
    dy = rand(numEffectors, 1) * ydiff;
    
    pos = [x0 + dx, y0 + dy];
end