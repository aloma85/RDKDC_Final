%% Final Project Main File
% 
% Steps: 
%     1. Dialog for user to choose which mode we are in (IK, RR, JT)
%     2. Move UR5 manually to the end position
%     3. Press button to log end position kinematics
%     4. Move UR5 manually to the start position
%     5. Press button to log start postition kinematics
%     6. Generate a trajectory depending on which mode we are in (IK, RR, JT)
%     7. Loop to traverse from start to end position

%%
ur5 = ur5_interface();
ur5.move_joints(ur5.home(), 20);
pause(6);

%%
g0s = [ROTZ(pi/2), [0; 0; 0.0892]; zeros(1, 3), 1];

%%
theta1 = [0; -pi/3; pi/4; 0; -pi/4; 0];
[g1, ~] = ur5FwdKin(theta1);
p2 = g1(1:3, 4) + [0; 0.3; 0];
g2 = [g1(1:3, 1:3), p2; zeros(1, 3), 1];

%%
frame1 = tf_frame('base_link', 'frame1', g1);
%%
frame2 = tf_frame('base_link', 'frame2', g2);

%%
q = ur5InvKin(g1);
[gapprox, ~] = ur5FwdKin(q(:, 1));
disp((g1 - gapprox));
%%
% theta1 = ur5InvKin(g1);
ur5.move_joints(theta1, 20);
pause(21);
v = 0.01; % m/s
drawLine(g1, g2, 0, v, ur5, theta1);