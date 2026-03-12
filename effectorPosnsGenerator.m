function effectorPosnsGenerator(xlim, ylim, filename, reset, N)

    % Randomly generates positions of deployed effectors within the deployable area
    %
    % Inputs
    % xlim: [xmin xmax]
    % ylim: [ymin ymax]
    % filename: MAT file to save bank
    % reset: overwrite existing bank if true
    % N: number of new positions to generate

    arguments
        xlim (1,2) double
        ylim (1,2) double
        filename char
        reset logical = false
        N (1,1) double {mustBeInteger,mustBePositive} = 10000
    end

    xmin = xlim(1);
    ymin = ylim(1);

    xspan = xlim(2) - xmin;
    yspan = ylim(2) - ymin;

    posns = zeros(N,2);

    hWaitbar = waitbar(0,...
        sprintf('New positions: 0 / %d',N),...
        'Name','Generating effector positions',...
        'CreateCancelBtn','delete(gcbf)');

    updateEvery = max(1, round(N/200));

    for i = 1:N

        posns(i,:) = [ xmin + rand()*xspan , ymin + rand()*yspan ];

        if ~ishandle(hWaitbar)

            disp("Generation stopped by user");

            posns = posns(1:i-1,:);

            break

        end

        if mod(i,updateEvery) == 0 || i == N

            waitbar(i/N,hWaitbar,...
                sprintf('New positions: %d / %d',i,N));

            drawnow limitrate

        end

    end

    if ishandle(hWaitbar)
        close(hWaitbar)
    end

    if isfile(filename)

        data = load(filename);

        if isfield(data,"effector_posns_bank") && ~reset

            effector_posns_bank = data.effector_posns_bank;

            effector_posns_bank = [effector_posns_bank ; posns];

        else

            effector_posns_bank = posns;

        end

    else

        effector_posns_bank = posns;

    end

    save(filename,"effector_posns_bank");

end
