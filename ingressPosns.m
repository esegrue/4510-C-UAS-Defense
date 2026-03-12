function pos = ingressPosns(xlim, ylim, numUAS, edges, minSep)

    % Randomly generates UAS ingress positions along the map perimeter
    %
    % Inputs
    % xlim: [xmin xmax]
    % ylim: [ymin ymax]
    % numUAS: number of adversaries
    % edges: "ALL","DISTRIBUTE","BOTTOM","RIGHT","TOP","LEFT"
    % minSep: minimum separation between ingress points

    arguments
        xlim (1,2) double
        ylim (1,2) double
        numUAS (1,1) double = 1
        edges (1,1) string = "ALL"
        minSep (1,1) double = 0
    end

    edges = upper(string(edges));

    xmin = xlim(1);
    xmax = xlim(2);
    ymin = ylim(1);
    ymax = ylim(2);

    width = xmax - xmin;
    height = ymax - ymin;

    pos = nan(numUAS,2);

    if minSep > 0

        count = 0;
        attempts = 0;
        maxAttempts = 2000;

        while count < numUAS && attempts < maxAttempts

            attempts = attempts + 1;

            edgeId = pickEdge(edges);

            p = samplePoint(edgeId,xmin,xmax,ymin,ymax);

            if count > 0

                d = vecnorm(pos(1:count,:) - p,2,2);

                if any(d < minSep)
                    continue
                end

            end

            count = count + 1;

            pos(count,:) = p;

        end

        if count < numUAS
            pos = pos(1:count,:);
        end

        return

    end

    if edges == "DISTRIBUTE"

        base = randperm(4);

        for i = 1:numUAS

            edgeId = base(mod(i-1,4)+1);

            pos(i,:) = samplePoint(edgeId,xmin,xmax,ymin,ymax);

        end

        return

    end

    if edges == "ALL"

        perimeterLen = 2*width + 2*height;

        d = rand(numUAS,1) * perimeterLen;

        idx = d < width;

        pos(idx,1) = xmin + d(idx);
        pos(idx,2) = ymin;

        idx = (d >= width) & (d < width + height);

        pos(idx,1) = xmax;
        pos(idx,2) = ymin + (d(idx) - width);

        idx = (d >= width + height) & (d < 2*width + height);

        pos(idx,1) = xmax - (d(idx) - (width + height));
        pos(idx,2) = ymax;

        idx = d >= (2*width + height);

        pos(idx,1) = xmin;
        pos(idx,2) = ymax - (d(idx) - (2*width + height));

    else

        edgeId = pickEdge(edges);

        switch edgeId

            case 1
                pos(:,1) = xmin + rand(numUAS,1)*width;
                pos(:,2) = ymin;

            case 2
                pos(:,1) = xmax;
                pos(:,2) = ymin + rand(numUAS,1)*height;

            case 3
                pos(:,1) = xmin + rand(numUAS,1)*width;
                pos(:,2) = ymax;

            case 4
                pos(:,1) = xmin;
                pos(:,2) = ymin + rand(numUAS,1)*height;

        end

    end

end

function edgeId = pickEdge(edges)

    if edges == "ALL" || edges == "DISTRIBUTE"
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
            error("Invalid edge selection")

    end

end

function p = samplePoint(edgeId,xmin,xmax,ymin,ymax)

    width = xmax - xmin;
    height = ymax - ymin;

    switch edgeId

        case 1
            p = [xmin + rand()*width , ymin];

        case 2
            p = [xmax , ymin + rand()*height];

        case 3
            p = [xmin + rand()*width , ymax];

        case 4
            p = [xmin , ymin + rand()*height];

    end

end
