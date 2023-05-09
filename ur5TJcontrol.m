function TJ = ur5TJcontrol(g_d, K, ur5, dt, mode)
    % ur5TJcontrol: Control UR5 to achieve the desired end-effector pose
    % using the transpose Jacobian method.
    % Input:
    %   g_d: 4x4 desired end-effector pose
    %   K: control gain
    %   ur5: UR5 robot object

    % Initialize variables
    T = 0.4;
    q = ur5.get_current_joints();
    g = ur5FwdKin(q);
    xi = getXi(g_d \ g);
    v = xi(1:3);
    w = xi(4:6);
    m = 10;
    gk = g;
    % Show desired frame
%     Frame_desired = tf_frame('base_link', 'Frame_desired', g_d);

    % Continue until the desired pose is reached
%     while or(norm(v) > 0.05, norm(w) > pi/50)
    while norm(g_d-gk) > 0.01
        qk = ur5.get_current_joints();
        gk = ur5FwdKin(qk);
        J = ur5BodyJacobian(qk);
        xi = getXi(g_d \ gk);
        v = xi(1:3);
        w = xi(4:6);
        if mode == 2
            dq = K * (J' * xi);
        elseif mode == 1
            dq = K * (J \ xi);
        end

%         % Adjust control input
%         if v > 0.5
%             xi = xi / 4;
%         end
%         if norm(xi) < 0.6
%             T = (xi' * J * J' * xi) / norm(J * J' * xi);
%         end

        % Scale dq based on its magnitude
%         if 0.0001 < norm(dq) && norm(dq) < 0.02
%             dq = dq * 0.02 / norm(K * (J' * xi));
%             m = 5;
%         elseif norm(dq) < 0.0001
%             dq = dq * 0.001 / norm(K * (J' * xi));
%             m = 3;
%         end

        % Update joint angles
        qk = qk - dq;

        % Check manipulability
        if abs(manipulability('sigmamin', J)) < 0.0001
            finalerr = -1;
            return;
        end

        % Move robot and wait for motion to complete
        t = max(abs(dq) / (ur5.speed_limit * pi)) * m;
        ur5.move_joints(qk, dt);
        pause(dt);
    end
end
