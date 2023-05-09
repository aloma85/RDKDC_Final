function TJ = ur5TJcontrol(g_d, K, ur5, dt)
    % ur5TJcontrol: Control UR5 to achieve the desired end-effector pose
    % using the transpose Jacobian method.
    % Input:
    %   g_d: 4x4 desired end-effector pose
    %   K: control gain
    %   ur5: UR5 robot object

    % Initialize variables
    T = 1;
    qk = ur5.get_current_joints();
    gk = ur5FwdKin(qk);
    gbetween = inv(gk)*g_d
    which getXi
    xi = getXi(gbetween)
    J = ur5BodyJacobian(qk);
%     xi = xi / norm(xi);
    v = xi(1:3);
    w = xi(4:6);
    m = 10;

    % Show desired frame
%     Frame_desired = tf_frame('base_link', 'Frame_desired', g_d);

    % Continue until the desired pose is reached
%     while or(norm(v) > 0.003, norm(w) > pi/180)
%     norm(xi)
    while norm(g_d-gk) > 0.01
%         xinorm = norm(xi);
%         qk = ur5.get_current_joints();
%         gk = ur5FwdKin(qk);
%         J = ur5BodyJacobian(qk);
%         xi = getXi(inv(gk)*g_d)
% %         xi = xi / norm(xi);
%         v = xi(1:3);
%         w = xi(4:6);
        dq = K * T * (J' * xi);
%         disp(dq);
        

        % Adjust control input
        if norm(v) > 0.5
            xi = xi / 4;
        end
        if norm(xi) < 0.6
            T = (xi' * J * J' * xi) / norm(J * J' * xi);
        end

%         Scale dq based on its magnitude
        if 0.0001 < norm(dq) && norm(dq) < 0.02
            dq = dq * 0.02 / norm(K * T * (J' * xi));
            m = 5;
        elseif norm(dq) < 0.0001
            dq = dq * 0.001 / norm(K * T * (J' * xi));
            m = 3;
        end
%         disp(dq);
        % Update joint angles
        disp(dq);
        qk = qk + dq;
%         disp(qk);
        % Check manipulability
        if abs(manipulability("sigmamin", J)) < 0.0001
            finalerr = -1;
            return;
        end

        % Move robot and wait for motion to complete
        t = max(abs(dq) / (ur5.speed_limit * pi)) * m
%         disp(t);
        ur5.move_joints(qk, t);
        pause(t);
        disp('here');
        qk = ur5.get_current_joints();
        gk = ur5FwdKin(qk);
        J = ur5BodyJacobian(qk);
        xi = getXi(inv(gk)*g_d)
%         xi = xi / norm(xi);
        v = xi(1:3);
        w = xi(4:6);
    end
end
