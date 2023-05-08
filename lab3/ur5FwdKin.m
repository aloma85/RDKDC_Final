function [gst,Twist] = ur5FwdKin(q)
% – Inputs: q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4, θ5, θ6]T where θn is the angle of joint n for n = 1,··· ,6. Be careful of sign convention!
% – Output: gst: end effector pose, gst (4 × 4 matrix)
    % Define lengths in meters
%     l = [0.0892, 0.425, 0.392, 0.1093, 0.09475, 0.0825];
    l = [0.425, 0.392, 0.1093, 0.09475, 0.0825];

    % w
%     ey = [0; 1; 0];
%     ez = [0; 0; 1];
%     w = [ez, ey, ey, ey, -ez, ey];
    ex = [1; 0; 0];
    ez = [0; 0; 1];
    w = [ez, ex, ex, ex, -ez, ex];

    % q
%     q_coords = [0, 0, 0;
%                 0, 0, l(1);
%                 l(2), 0, l(1);
%                 l(2) + l(3), 0, l(1);
%                 l(2) + l(3), l(4), l(1) - l(5);
%                 l(2) + l(3), l(4) + l(6), l(1) - l(5)].';
    q_coords = [0, 0, 0;
                0, 0, 0;
                0, -l(1), 0;
                0, -l(1)-l(2), 0;
                l(3), -l(1)-l(2), 0;
                0, -l(1)-l(2), -l(4)].';

    % Calculate twists
    Twist = [cross(q_coords, w); w];
   
    % Calculate gst0
%     gst0 = [ROTX(-pi/2)*ROTZ(pi), q_coords(:, 6); 0 0 0 1];
    gst0 = [ROTX(pi), [l(3)+l(5); -l(1)-l(2); -l(4)]; 0 0 0 1];


     % skew3
    skew3 = @(Vs) [0, -Vs(3), Vs(2); Vs(3), 0, -Vs(1); -Vs(2), Vs(1), 0];
     % wedge
    wedge = @(Twist) [skew3(Twist(4:6)), Twist(1:3); 0 0 0 0];

    % Calculate exponentials of wedges
    g = eye(4);
    for i = 1:6
        exp_wedge = expm(wedge(Twist(:, i)) * q(i));
        g = g * exp_wedge;
    end
    
    % Calculate gst
    gst = g * gst0;
    
end
