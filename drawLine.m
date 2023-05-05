function drawLine(g1, g2, mode, speed, ur5, thetastart)
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
    theta = thetastart;
    tstart = tic;
    t = tic;
    T = norm(p2-p1) / speed;
    dt = 0.1;
    while (toc(tstart) < T)
        tnext = toc(t);
        if (tnext >= dt)
            snew = speed * toc(tstart);
            pnew = p1 + (snew / sfinal)*(p2 - p1);
            % find new g based on xnew
            gnew = [g1(1:3, 1:3), pnew; zeros(1, 3), 1];
            % get theta from new g
            thetanew = ur5InvKin(gnew);
            % Find theta with least squared distance from current theta
            dists = zeros(1, size(thetanew, 2));
            for i=1:size(thetanew, 2)
                dists(i) = norm(thetanew(:, i) - theta);
%                 disp(dists(i));
            end
            [~, idx] = min(dists);
%             disp(thetanew(:, idx) - theta);
%             disp(thetanew);
%             disp(theta);
            theta = thetanew(:, idx);
            ur5.move_joints(theta, dt);
            t = tic;
        end
    end
end