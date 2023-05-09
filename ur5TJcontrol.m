function safety = ur5TJcontrol(g_d, K, ur5, dt, mode, vdes)
    % ur5TJcontrol: Control UR5 to achieve the desired end-effector pose
    % using the transpose Jacobian method.
    % Input:
    %   g_d: 4x4 desired end-effector pose
    %   K: control gain
    %   ur5: UR5 robot object

    % Initialize variables
    q = ur5.get_current_joints();
    gk = ur5FwdKin(q);

    % Continue until the desired pose is reached
    while norm(g_d-gk) > 0.005
%         disp('here');
        qk = ur5.get_current_joints();
        gk = ur5FwdKin(qk);
        J = ur5BodyJacobian(qk);
        maxxi = getXi(g_d \ gk);
        xi = maxxi * vdes*dt / norm(maxxi);
        if norm(xi) > norm(maxxi)
            xi = maxxi;
        end
        if mode == 2
            dq = K * (J' * xi);
        elseif mode == 1
            dq = (J \ xi);
        end

        % Update joint angles
        qk = qk - dq;
        safety = safetyCheck(qk);
        if (safety == 0)
            return;
        end

        % Move robot and wait for motion to complete
        ur5.move_joints(qk, dt);
        pause(dt);
    end
end
