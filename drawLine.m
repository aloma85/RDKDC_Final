function safety = drawLine(g1, g2, mode, speed, ur5, thetastart)
% Inputs: g1 - starting position (homogenous coordinates)
%         g2 - end position (homogenous coordinates)
%         mode - integer indicating the tracking mode (0-IK, 1-RR, 2-JT)
%         speed - cartesian speed we would like to move at
%         ur5 - the ur5 interface object
% Given the endpoints of a line, tracks the trajectory as a line between
% them using the method specified by mode.

% IK:
% - Generate intermediate gmatrices
% - 
    % Get function of x(t)
    % from g1, get p1
    p1 = g1(1:3, 4);
    % from g2, get p2
    p2 = g2(1:3, 4);
    
    % compute total distance traveled
    sfinal = norm(p2 - p1); % end position (1D)
    
    % Plan Trajectory
    T = norm(p2-p1) / speed;
    dt = T / 50; % T / 100
    t = 0:dt:T;

    if (mode == 0)
        disp("Calculating Inv Kin Trajectory ... be patient");
        theta_traj = zeros(6, length(t));
        theta_traj(:, 1) = thetastart;
        for i=2:length(t)
            snew = speed * t(i);
            pnew = p1 + (snew / sfinal)*(p2 - p1);
            % find new g based on xnew
            gnew = [g1(1:3, 1:3), pnew; zeros(1, 3), 1];
            % get theta from new g
            thetanew = ur5InvKin(gnew);
            % Find theta with least squared distance from current theta
            dists = zeros(1, size(thetanew, 2));
            for j=1:size(thetanew, 2)
                dists(j) = norm(thetanew(:, j) - theta_traj(:, i-1));
    %                 disp(dists(i));
            end
            [~, idx] = min(dists);
            theta_traj(:, i) = thetanew(:, idx);
        end
        disp('Executing Inv Kin Trajectory ...')
%         tsim = tic;
        for i=2:length(t)
            safety = safetyCheck(theta_traj(:, i));
            if (safety == 0)
                return;
            end
            ur5.move_joints(theta_traj(:, i), dt)
            pause(dt);
        end
%         toc(tsim)
    elseif (mode == 1)
        disp('Executing RR Control ...');
        safety = ur5TJcontrol(g2, 0.001, ur5, dt, mode, speed);
        if (safety == 0)
            return;
        end
    elseif (mode == 2)
        disp('Executing Transpose Jacobian Control ...');
        safety = ur5TJcontrol(g2, 1, ur5, dt, mode, speed);
        if (safety == 0)
            return;
        end
    end
end