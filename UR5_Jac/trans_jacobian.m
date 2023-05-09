function Jb_transpose = trans_jacobian(theta)
    % ur5BodyJacobianTranspose - Calculate the transpose of the body Jacobian
    % for a UR5 robot with given joint angles
    %
    % Input:
    %   theta - 6x1 vector of joint angles
    %
    % Output:
    %   Jb_transpose - Transpose of the 6x6 body Jacobian matrix

    % Define unit vectors
    e1 = [1, 0, 0]';
    e2 = [0, 1, 0]';
    e3 = [0, 0, 1]';

    % Define rotational axes
    w_1 = e3;
    w_2 = e2;
    w_3 = e2;
    w_4 = e2;
    w_5 = -e3;
    w_6 = e2;

    % Define reference points
    q1 = [0; 0; 0];
    q2 = [0; 0; 0.0892];
    q3 = [0.425; 0; 0.0892];
    q4 = [0.425 + 0.392; 0; 0.0892];
    q5 = [0.425 + 0.392; 0.1093; 0];
    q6 = [0.425 + 0.392; 0.1093; 0.0892 - 0.09475];

    % Define initial transformation matrix
    g_tool = [-1, 0, 0, 0.425 + 0.392;
          0, 0, 1, 0.0825 + 0.1093;
          0, 1, 0, 0.0892 - 0.09475;
          0, 0, 0, 1];

    % Calculate transformation matrices for each joint
    g1 = expm([SKEW3(w_1) -cross(w_1, q1); 0 0 0 0] * theta(1, 1));
    g2 = expm([SKEW3(w_2) -cross(w_2, q2); 0 0 0 0] * theta(2, 1));
    g3 = expm([SKEW3(w_3) -cross(w_3, q3); 0 0 0 0] * theta(3, 1));
    g4 = expm([SKEW3(w_4) -cross(w_4, q4); 0 0 0 0] * theta(4, 1));
    g5 = expm([SKEW3(w_5) -cross(w_5, q5); 0 0 0 0] * theta(5, 1));
    g6 = expm([SKEW3(w_6) -cross(w_6, q6); 0 0 0 0] * theta(6, 1));

    % Calculate forward kinematics
    f_kin = g1 * g2 * g3 * g4 * g5 * g6 * g_tool;

    % Calculate spatial twists
    Xi1 = [-cross(w_1, q1); w_1];
    Xi2 = [-cross(w_2, q2); w_2];
    Xi3 = [-cross(w_3, q3); w_3];
    Xi4 = [-cross(w_4, q4); w_4];
        Xi5 = [-cross(w_5, q5); w_5];
    Xi6 = [-cross(w_6, q6); w_6];

    % Calculate the spatial Jacobian
    spatial_j = [Xi1, Adj(g1) * Xi2, Adj(g1 * g2) * Xi3, Adj(g1 * g2 * g3) * Xi4, 
    Adj(g1 * g2 * g3 * g4) * Xi5, Adj(g1 * g2 * g3 * g4 * g5) * Xi6];

    % Calculate the body Jacobian
    Jb = inv(Adj(f_kin)) * spatial_j;

    % Calculate the transpose of the body Jacobian
    Jb_transpose = Jb';
end

