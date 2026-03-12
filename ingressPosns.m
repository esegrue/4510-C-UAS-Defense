function pos = ingressPosns(xlim, ylim, numUAS, edges, minSep)
% Inputs
% xlim: [xmin xmax] map x-limits
% ylim: [ymin ymax] map y-limits
% numUAS: number of UAS ingress points to generate
% edges: "ALL", "DISTRIBUTE", "BOTTOM", "RIGHT", "TOP", or "LEFT"
% minSep: minimum separation distance between ingress points (0 disables)
%
% Output
%
% Notes
% Points are generated on the map perimeter
% If minSep > 0, rejection sampling is used with a capped number of attempts

    arguments
        xlim (1, 2) double
        ylim (1, 2) double
        numUAS (1, 1) double = 1
        edges (1, 1) string = "ALL"
        minSep (1, 1) double = 0
    end

    edges = upper(string(edges));

    xmin = xlim(1);
    xmax = xlim(2);
    ymin = ylim(1);
    ymax = ylim(2);

    width = xmax - xmin;
    height = ymax - ymin;

    if minSep > 0
        pos = nan(numUAS, 2);

        count = 0;  % number of accepted points
        attempts = 0;  % total samples attempted
        maxAttempts = 2000;  % rejection sampling cap

        while count < numUAS && attempts < maxAttempts
            attempts = attempts + 1;

            edgeId = pickEdge(edges);
            p = samplePoint(edgeId, xmin, xmax, ymin, ymax);

            if count > 0
                d = vecnorm(pos(1:count, :) - p, 2, 2);
                if any(d < minSep)
                    continue
                end
            end

            count = count + 1;
            pos(count, :) = p;
        end

        if count < numUAS
            warning("ingressPosns:MinSepNotMet", "Could not place all ingress points with minSep constraint.");
            pos = pos(1:count, :);
        end

        return
    end

    pos = nan(numUAS, 2);

    if edges == "DISTRIBUTE"
        base = randperm(4);  % random ordering of [BOTTOM RIGHT TOP LEFT]
        edgeIds = zeros(numUAS, 1);  % edge assignment per UAS

        for i = 1:numUAS
            edgeIds(i) = base(mod(i - 1, 4) + 1);
        end

        for i = 1:numUAS
            pos(i, :) = samplePoint(edgeIds(i), xmin, xmax, ymin, ymax);
        end

        return
    end

    if edges == "ALL"
        perimeterLen = 2 * width + 2 * height;  % map perimeter length
        d = rand(numUAS, 1) * perimeterLen;  % distance along perimeter from bottom-left corner

        idx = d < width;
        pos(idx, 1) = xmin + d(idx);
        pos(idx, 2) = ymin;

        idx = (d >= width) & (d < (width + height));
        pos(idx, 1) = xmax;
        pos(idx, 2) = ymin + (d(idx) - width);

        idx = (d >= (width + height)) & (d < (2 * width + height));
        pos(idx, 1) = xmax - (d(idx) - (width + height));
        pos(idx, 2) = ymax;

        idx = d >= (2 * width + height);
        pos(idx, 1) = xmin;
        pos(idx, 2) = ymax - (d(idx) - (2 * width + height));
    else
        edgeId = pickEdge(edges);

        switch edgeId
            case 1
                pos(:, 1) = xmin + rand(numUAS, 1) * width;
                pos(:, 2) = ymin;
            case 2
                pos(:, 1) = xmax;
                pos(:, 2) = ymin + rand(numUAS, 1) * height;
            case 3
                pos(:, 1) = xmin + rand(numUAS, 1) * width;
                pos(:, 2) = ymax;
            case 4
                pos(:, 1) = xmin;
                pos(:, 2) = ymin + rand(numUAS, 1) * height;
        end
    end
end

function edgeId = pickEdge(edges)
% Inputs
% edges: edge selection string
%
% Output
%
% Notes
% Returns numeric edge id: 1 BOTTOM, 2 RIGHT, 3 TOP, 4 LEFT

    if edges == "ALL"
        edgeId = randi(4);
        return
    end

    if edges == "DISTRIBUTE"
        edgeId = randi(4);
        return
    end

    switch edges
        case "BOTTOM"
            edgeId = 1;
        case "RIGHT"
            edgeId = 2;
        case "TOP"
            edgeId = 3;
        case "LEFT"
            edgeId = 4;
        otherwise
            error("ingressPosns:BadEdge", "Invalid edge selection: %s", edges);
    end
end

function p = samplePoint(edgeId, xmin, xmax, ymin, ymax)
% Inputs
% edgeId: numeric edge id (1 BOTTOM, 2 RIGHT, 3 TOP, 4 LEFT)
% xmin, xmax, ymin, ymax: map bounds
%
% Output
%
% Notes
% Returns a single [x y] point sampled uniformly on the selected edge

    width = xmax - xmin;
    height = ymax - ymin;

    switch edgeId
        case 1
            p = [xmin + rand() * width, ymin];
        case 2
            p = [xmax, ymin + rand() * height];
        case 3
            p = [xmin + rand() * width, ymax];
        case 4
            p = [xmin, ymin + rand() * height];
    end
end
