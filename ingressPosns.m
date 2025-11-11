% Generates N evenly spaced entrances around L x L map perimeter

    % Input:
        % N - number of entrance points
        % L - side length of the square map
    % Output:
        % entrances - N x 2 matrix of [x, y] entrance coordinates

function entrances = ingressPosns(N, L)

    L = L - 0.1; %You don't want to start at edge per se, but slightly off
    P = 4*L;                              % total perimeter

    s = linspace(0, P, N+1).';            % evenly spaced distances along the perimeter
    s(end) = [];                          % drop duplicate endpoint

    x = zeros(N,1);                       % preallocate
    y = zeros(N,1);

    idx = s < L;                          % bottom edge (0 ≤ s < L
    x(idx) = .05 + s(idx);
    y(idx) = 0.05;
       
    idx = (s >= L) & (s < 2*L);           % right edge (L ≤ s < 2L)
    x(idx) = L;
    y(idx) = s(idx) - L + 0.05;

    idx = (s >= 2*L) & (s < 3*L);         % top edge (2L ≤ s < 3L)
    x(idx) = L - (s(idx) - 2*L) + 0.05;
    y(idx) = L;

    idx = (s >= 3*L);                     % left edge (3L ≤ s < 4L)
    x(idx) = 0.05;
    y(idx) = L - (s(idx) - 3*L);

    entrances = [x, y];
end