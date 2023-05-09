% Inverse Kinematics Test
rosshutdown;
ur5 = ur5_interface();
joint_offset = [-pi/2 -pi/2 0 -pi/2 0 0]';
joints = [0 0 0 0 0 0]';
ur5.move_joints(joints, 5);
pause(6);
g_S_T = ur5FwdKin(joints);
% g_Sprime_S = [ROTZ(pi), [0; 0; 0]; 0, 0, 0, 1]; %
% g_S_T = g_Sprime_S*g_S_T; %

g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  %transformation from keating base to {S}
g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1]; % tool k is inline with tool 0
g_baselink_baseK = [ROTZ(pi), [0; 0; 0]; 0, 0, 0, 1];
tf_frame('base_link', 'baseK', g_baselink_baseK);
pause(0.5);
tf_frame('baseK', 'S', g_baseK_S);
pause(0.5);
tf_frame('tool0', 'T', inv(g_T_toolK));
pause(0.5);

tf_frame('S', 'starting', g_S_T);
pause(0.5);
% g_S_T(1,4) = g_S_T(1,4)+0.2;
% temp = g_S_T(2, 4);
% g_S_T(2, 4) = g_S_T(1, 4);
% g_S_T(1, 4) = temp;
g_S_T(1, 4) = g_S_T(1, 4) + 0.2;
g_S_T(2, 4) = g_S_T(2, 4) + 0.4;
% g_S_T = [ROTZ(pi/2), [0; 0; 0]; 0, 0, 0, 1]*g_S_T;
tf_frame('S', 'desired', g_S_T);
pause(0.5);
% %%
%
% 
% %%
% g_baselink_baseK = [ROTZ(pi), [0; 0; 0]; 0, 0, 0, 1]; %
% g_baselink_S = g_baselink_baseK * g_baseK_S; %
%-90 degree rotation around z and up x 0.0892 
% tf_frame('base_link','S',g_baselink_S)
% pause(0.5)
% baseKFrame = tf_frame('S','base_K',eye(4));
% pause(0.5)
% baseKFrame.move_frame('S',inv(g_baseK_S));
% pause(0.5)
% 
% tf_frame('S', 'starting', g_S_T);
% pause(0.5);
% g_S_T(1,4) = g_S_T(2,4)+0.1;
% tf_frame('S', 'desired', g_S_T);
% pause(0.5);

% g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1]; %transformation from {T} to keating tool 
% %-90 around x and 90 around y
% tf_frame('tool0','T',inv(g_T_toolK));
% pause(0.5)

% toolKFrame = tf_frame('T','tool_K',eye(4));
% pause(0.5)
% toolKFrame.move_frame('T',g_T_toolK);
% pause(0.5)
g_des = g_baseK_S*g_S_T*g_T_toolK; %transformation from keating base to keating tool 
tf_frame('baseK','T2',g_des);
pause(0.5)

thetas = ur5InvKin(g_S_T);
thetas(1, :) = thetas(1, :) + pi;
thetas(2, :) = thetas(2, :) - pi/2;
thetas(4, :) = thetas(4, :) - pi/2;
for i=1:6
    for j=1:8
        if thetas(i,j) <= -pi
            thetas(i,j) = thetas(i,j) + 2*pi;
        elseif thetas(i,j) > pi
            thetas(i,j) = thetas(i,j) - 2*pi;
        end
    end
end
ur5.move_joints(thetas(:,1),3)
