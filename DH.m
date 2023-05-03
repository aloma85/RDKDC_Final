function [G] = DH(a, alpha, d, theta)
% DH - Denavit-Hartenberg Transformation Matrix Calculation
%   Computes the transformation matrix using the provided DH parameters
%   following the traditional definition.
%
% Inputs:
%   a - Link length (distance along the x-axis)
%   alpha - Link twist (rotation about the x-axis)
%   d - Link offset (distance along the z-axis)
%   theta - Joint angle (rotation about the z-axis)
%
% Output:
%   G - 4x4 homogeneous transformation matrix

% Initialize the individual transformation matrices
Td = eye(4);
Ta = eye(4);
Rtheta = eye(4);
Ralpha = eye(4);

% Assign the given values to the matrices
Td(3, 4) = d;
Ta(1, 4) = a;
Rtheta(1:3, 1:3) = ROTZ(theta);
Ralpha(1:3, 1:3) = ROTX(alpha);

% Compute the combined transformation matrix G
G = Td * Rtheta * Ta * Ralpha;
end
