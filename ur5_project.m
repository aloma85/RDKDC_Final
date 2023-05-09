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
ur5.move_joints(ur5.home(), 5);
pause(6);

%%
g_baselink_S = [ROTZ(pi/2) [0 0 0.0892]'; 0 0 0 1];  
%-90 degree rotation around z and up x 0.0892 
tf_frame('base_link','S',g_baselink_S);
pause(0.5);

%transformation from {T} to keating tool, g_T->6
% g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
g_tool0_T = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
%-90 around x and 90 around y
tf_frame('tool0','T',g_tool0_T);
pause(0.5);

% Tool frame to the pen tip 
% g_T_tip = [eye(3), [0.12228, 0, 0.049]'; 0,0,0,1];

% g_S_tip = g_S_T * g_T_tip
% g_S_T = ur5FwdKin(joints);
% g_S_tip = g_S_T * g_T_tip;
% tip_frame = tf_frame('S','tip',g_S_tip);
% pause(0.5);
%%
g1 = ur5.get_current_transformation('S', 'T');
theta1 = ur5InvKin(g1);
%%
ur5.move_joints(theta1(:, 1), 5);
%%
g1 = [eye(3), [0.5; 0.5; 0]; zeros(1, 3), 1];
% g1 = ur5FwdKin(ur5.get_current_joints());
% g1 = ur5.get_current_transformation('S', 'T');
g2 = [eye(3), [0.5; 0.4; 0]; zeros(1, 3), 1];
% theta1 = ur5InvKin(g1);
% g_approx = zeros(1, size(theta1, 2));
% for i=1:size(theta1, 2)
% %     for j=1:size(theta1, 1)
% %         newtheta = theta1(:, i);
% %         newtheta(j) = -newtheta(j)
% %         g1app_switch = ur5FwdKin(newtheta);
% %         gapprox = norm(g1)
%     g1app = ur5FwdKin(theta1(:, i));
%     g_approx(i) = norm(g_baseK_S*(g1app-g1)*g_T_toolK);
% end
% [~, idx] = min(g_approx);

% theta1 = theta1(:, 2);
% gapprox = g_T_toolK*ur5FwdKin(theta1)*g_baseK_S;
% disp(g1-gapprox);
%%
moving = tf_frame("S", "moving", inv(g_baselink_S)*g1);
%%
frame1 = tf_frame('S', 'frame1', g1);
%%
frame2 = tf_frame('S', 'frame2', g2);
%%
ur5 = ur5_interface();
ur5.move_joints(zeros(6, 1), 5);
pause(6);
% q1 = ur5.get_current_joints();
g1 = ur5.get_current_transformation("base_link", "tool0");

%%
% ur5 = ur5_interface();
% ur5.move_joints(ur5.home(), 10);
% pause(11);
% g1 = [eye(3), [0.5; 0.2; 0]; zeros(1, 3), 1];
% g1 = ur5.get_current_transformation("baseK", "tool0");
theta1 = ur5InvKin(g1);
% g1approx = ur5FwdKin(theta1(:, 1))
ur5.move_joints(theta1(:, 1), 5);
pause(6);
% theta1(:, 1);
% qnow = ur5.get_current_joints();
% gnow = ur5.get_current_transformation("S", "T")
% g1-gnow
% qnow-theta(:, 1)
v = 0.01; % m/s
drawLine(g1, g2, 0, v, ur5, theta1(:, 1));
%%
q = ur5.get_current_joints()
gnow = ur5FwdKin(q);
q_approx = ur5InvKin(gnow)
%%
ur5.move_joints(theta1(:, 1), 5);
pause(6);
ur5TJcontrol(g2, 0.01, ur5);
