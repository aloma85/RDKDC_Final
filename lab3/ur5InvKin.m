function [ theta ] = ur5InvKin( gd )
    % ur5InvKin: Computes the inverse kinematics of a UR5 robot
    %
    % Input args:
    %   gd - the 4x4 matrix - desired transformation from base to end
    % Output args:
    %   theta - 6x8 matrix, each column represents one possible solution 
    %           of joint angles

    % Validate input size
    if ~isequal(size(gd), [4,4])
        error('Input transform must be a 4x4 matrix');
    end

    % Define base and tool frame adjustments
    g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];
    g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
    gd = g_baseK_S*gd*g_T_toolK;

    % Initialize theta matrix
    theta = zeros(6, 8);

    % DH parameters
    d = [0.08916, 0, 0, 0.10915, 0.09465, 0.0823];
    a = [0, -0.425, -0.392, 0, 0, 0];
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    % Calculate joint angles (theta1, theta5, theta6, theta3, theta2, theta4)
    % ...

    % Apply offset to joint angles
    offset = [0;pi/2;0;pi/2;0;0];
    for j=1:8
        theta(:,j) = theta(:,j)+offset;
    end
    
    for j=1:8
        theta(1,j) = theta(1,j)-pi;
    end

    % Bound the joint angles from -pi to pi
    for i=1:6
        for j=1:8
            if theta(i,j) <= -pi
                theta(i,j) = theta(i,j) + 2*pi;
            elseif theta(i,j) > pi
                theta(i,j) = theta(i,j) - 2*pi;
            end
        end
    end
end
