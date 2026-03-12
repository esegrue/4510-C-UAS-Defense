function effectorPosnsGenerator(xlim, ylim, filename, reset, N)
% Inputs
% xlim: [xmin xmax] deployable area x-limits
% ylim: [ymin ymax] deployable area y-limits
% filename: MAT-file path to save effector_posns_bank into
% reset: true overwrites existing bank, false appends when possible
% N: number of new positions to generate (positive integer)
%
% Output
%
% Notes
% If filename exists and reset is false, new positions append to the existing bank
% Close the waitbar window to cancel generation early

    arguments
        xlim (1, 2) double
        ylim (1, 2) double
        filename char
        reset logical = false
        N (1, 1) double {mustBeInteger, mustBePositive} = 10000
    end

    xmin = xlim(1);  % deployable area x-min
    ymin = ylim(1);  % deployable area y-min
    xspan = xlim(2) - xmin;  % deployable area width
    yspan = ylim(2) - ymin;  % deployable area height

    posns = zeros(N, 2);  % generated [x y] positions

    hWaitbar = waitbar(0, sprintf('New positions: 0 / %d', N), 'Name', 'Generating effector positions', 'CreateCancelBtn', 'delete(gcbf)');

    updateEvery = max(1, round(N / 200));  % UI update cadence

    for i = 1:N
        posns(i, :) = [xmin + rand() * xspan, ymin + rand() * yspan];

        if ~ishandle(hWaitbar)
            disp('Generation stopped.');
            posns = posns(1:i-1, :);
            break
        end

        if mod(i, updateEvery) == 0 || i == N
            waitbar(i / N, hWaitbar, sprintf('New positions: %d / %d', i, N));
            drawnow limitrate
        end
    end

    if ishandle(hWaitbar)
        close(hWaitbar)
    end

    if isfile(filename)
        data = load(filename);
        if isfield(data, 'effector_posns_bank') && (reset == false)
            effector_posns_bank = data.effector_posns_bank;
            effector_posns_bank = [effector_posns_bank; posns];
        else
            effector_posns_bank = posns;
        end
    else
        effector_posns_bank = posns;
    end

    save(filename, 'effector_posns_bank');
end
