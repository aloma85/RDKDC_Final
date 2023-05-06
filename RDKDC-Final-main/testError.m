% gd=[0,1,0,0.3; -1,0,0,-0.4; 0,0,-1,0.22; 0,0,0,1];
% g=[0,-1,0,0.9; -1,0,0,-0.4; 0,0,-1,0.22; 0,0,0,1];
% Error(gd,g)

function [dso3, dR3] = testError(gd, g)
% gd is the desired start location 
% g  = (R, r) is the actual start location that the robot end-effector reaches

% get the rotation matrix 
Rd = gd(1:3, 1:3);
rd = gd(1:3,4);

% get the point 
R = g(1:3, 1:3);
r = g(1:3,4);

% calculate the error
dso3 = sqrt(trace((R-Rd)*(R-Rd).'))
dR3 = norm(r - rd)

end