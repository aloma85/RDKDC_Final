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
disp('Setting up UR5 and necessary frames ...');
ur5 = ur5_interface();
ur5.move_joints(ur5.home(), 5);
pause(6);
%% For user-inputted gstart, gend
g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  %transformation from keating base to {S}
g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1]; % tool k is inline with tool 0
g_baselink_baseK = [ROTZ(pi), [0; 0; 0]; 0, 0, 0, 1];
tf_frame('base_link', 'baseK', g_baselink_baseK);
pause(0.5);
tf_frame('baseK', 'S', g_baseK_S);
pause(0.5);
tf_frame('tool0', 'T', inv(g_T_toolK));
pause(0.5);

while (1)
    mode = input("Enter a control mode: 0 - Inverse Kinematics, 1 - Resolved Rate, 2 - Transpose Jacobian\n");
    try 
        if (0 <= mode && mode <= 2)
            break
        else
            disp("Please enter 0, 1, or 2.");
        end
    catch
        disp("Please enter a valid integer.");
    end
end

input("Move UR5 to ending position and press enter.");
gend = ur5.get_current_transformation('S', 'T');
input("Move UR5 to starting position and press enter.");
gstart = ur5.get_current_transformation('S', 'T');
%% For Custom gstart, gend
gstart = [0, 0, -1, 0.3; 0, -1, 0, -0.4; -1, 0, 0, 0.22; 0, 0, 0, 1];
gend = [0, 0, -1, -0.3; 0, -1, 0, 0.39; -1, 0, 0, 0.22; 0, 0, 0, 1];
qs = ur5InvKin(gstart);
ur5.move_joints(qs(:, 1), 5);
%%
if mode == 0 || mode == 1
    g1 = gstart;
    p = gend(1:2, 4) - gstart(1:2, 4);
    [~, idx1] = max(abs(p));
    [~, idx2] = min(abs(p));
    g1(idx1, 4) = g1(idx1, 4) + 2/3*(gend(idx1, 4) - gstart(idx1, 4));
    g2 = g1;
    g2(idx2, 4) = g2(idx2, 4) + (gend(idx2, 4) - gstart(idx2, 4));
    gend = [g2(1:3, 1:3), [gend(1:2, 4); g2(3, 4)]; 0 0 0 1];
    
    tf_frame('S', 'gstart', gstart);
    pause(0.5);
    tf_frame('S', 'g1', g1);
    pause(0.5);
    tf_frame('S', 'g2', g2);
    pause(0.5);
    tf_frame('S', 'gend', gend);
    pause(0.5);

    v = 0.05;
    safety = drawLine(gstart, g1, mode, v, ur5, ur5.get_current_joints());
    if (safety == 0)
        return;
    end
    disp('here');
    safety = drawLine(g1, g2, mode, v, ur5, ur5.get_current_joints());
    if (safety == 0)
        return;
    end
    safety = drawLine(g2, gend, mode, v, ur5, ur5.get_current_joints());
    if (safety == 0)
        return;
    end
else
    v = 0.1;
    gend = [gstart(1:3, 1:3), [gend(1:2, 4); gstart(3, 4)]; 0 0 0 1];
    safety = drawLine(gstart, gend, mode, v, ur5, ur5.get_current_joints());
    if (safety == 0)
        return;
    end
end

