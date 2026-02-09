function effectorPosnsGenerator(xlim, ylim, filename, reset)
    % Randomly generates positions of all deployed effectors within the
    % deployable area

    % Inputs -
    % xlim: [a,b] width of deployable area
    % ylim: [c,d] height of deployable area
    % filename: .mat file to save to

    % Outputs -
    % filename: updated position bank
    
    arguments
        xlim (1,2) double
        ylim (1,2) double
        filename char
        reset logical = false
    end
    
    x0 = xlim(1);
    y0 = ylim(1);
    xdiff = xlim(2) - x0;
    ydiff = ylim(2) - y0;
    posns = [0,0];
    
    i = 1;
    hWaitbar = waitbar(0, 'New positions: 1', 'Name', 'Generating effector positions','CreateCancelBtn','delete(gcbf)');
    while true
        dx = rand()*xdiff;
        dy = rand()*ydiff;
        posns(end+1,:) = [x0 + dx, y0 + dy];
        if ~ishandle(hWaitbar)
            disp('Generation stopped by user.')
            break
        else
            waitbar(i/(i+1000),hWaitbar, ['New positions: ' num2str(i)]);
            i = i + 1;
        end
        pause(0.01)
    end

    if isfile(filename)
        data = load(filename);
        if isfield(data, 'effector_posns_bank') && (reset == false)
            effector_posns_bank = data.('effector_posns_bank');
            effector_posns_bank = [effector_posns_bank; posns]; 
        else
            effector_posns_bank = posns;
        end           
    else
        effector_posns_bank = posns;
    end
    save(filename, 'effector_posns_bank');
end