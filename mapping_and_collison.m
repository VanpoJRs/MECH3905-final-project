%   x_current: current step
%   x_predict: RK4 predicted step (next step)
%   mask: 1=allowed, 0=wall
%   substeps: number of small steps for safe movement

function x_next = limitMovement_RK4rollback(x_current, x_predict, mask, substeps)

    % Start from current state
    x_next = x_current;

    % Total displacement from RK4
    dx_total = x_candidate(1) - x_current(1);
    dy_total = x_candidate(2) - x_current(2);

    % Divide into small steps, makes the steps accurate so the moving obj won't skip the wall
    dx = dx_total/substeps;
    dy = dy_total/substeps;

    % Get mask size
    [rows, cols] = size(mask);

    % Step-by-step movement
    for k = 1:substeps

        % Try small move
        x_trial = x_next;
        x_trial(1) = x_trial(1) + dx;
        x_trial(2) = x_trial(2) + dy;

        % Convert to pixel indices
        col = round(x_trial(1));
        row = round(x_trial(2));

        % Boundary check
        if row < 1 || row > rows || col < 1 || col > cols
            break
        end

        % Mask check
        if mask(row, col) == 1
            x_next = x_trial;
        else
            break % Collision happens and stop moving
        end

    end

    % Keep RK4 velocity only if movement succeeded
    if isequal(x_next(1:2), x_current(1:2))
        x_next(3) = 0;
        x_next(4) = 0;
    else
        x_next(3) = x_candidate(3);
        x_next(4) = x_candidate(4);
    end

end
