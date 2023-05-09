function Ad = Adj(g)
    % Adj - Calculate the Adjoint representation of a transformation matrix
    %
    % Input:
    %   g - 4x4 homogeneous transformation matrix
    %
    % Output:
    %   Ad - 6x6 Adjoint representation of the input transformation matrix

    % Extract the rotation matrix (R) from the transformation matrix
    R = g(1:3, 1:3);

    % Extract the translation vector (p) from the transformation matrix
    p = g(1:3, 4);

    % Compute the skew-symmetric matrix of the translation vector (p)
    skew_p = SKEW3(p);

    % Calculate the Adjoint representation (Ad)
    Ad = [R, skew_p * R;
          zeros(3, 3), R];
end
