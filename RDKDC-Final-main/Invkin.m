function Invkin(qs,qt,ur5)
%Get the joints of Start Frame
gs = ur5FwdKin(qs);
%Get the joints of Target Frame
gt = ur5FwdKin(qt);
%The right above distance d from target and end position on (z)
d = 0.1;
% The time
t=10;
% set the above of gs frame and above of gt frame
gsabove = gs;
gsabove(3,4) = gsabove(3,4) + d;
gtabove=gt;
gtabove(3,4) = gtabove(3,4) + d;
%% step 0: Move UR5 to the home configuration
ur5.move_joints(ur5.home,t);
pause(t);
%% step 1: move to the left above the start location.
    % Get the joint of the left side
    q1above = ur5InvKin(gsabove);
   
    %Chose a suitalbe joint set
    q1above_chosen= selectJoint(q1above, ur5);
    ur5.move_joints(q1above_chosen,t);
    pause(t);
    display("step 1 ");
   
    %step 2: move straight down to the start location
    q1=ur5InvKin(gs);
    q1_chosen = selectJoint(q1, ur5);
    ur5.move_joints(q1_chosen,t);
    pause(t);
    display("step 2 ");
        pause(0.5);
        q_recent = ur5.get_current_joints();
        g_recent = ur5FwdKin(q_recent);
    error=testError(gs,g_recent);
    fprintf('error of invkin:')
    disp(error);
    
%% step 3: move back to the previous pose
    ur5.move_joints(q1above_chosen,t);
    pause(t);
    % ur5.move_joints(ur5.home,t);
    % pause(t/4);
    display("step 3 ");
%% step 4: move to the right above the target location
    q2above = ur5InvKin(gtabove);
    q2above_chosen = selectJoint(q2above, ur5);
    ur5.move_joints(q2above_chosen, t/2);
    pause(t);
    display("step 4 ");
%%  step 5: move straight down to the target location and then move back to this pose
    q2 = ur5InvKin(gt);
    q2_chosen = selectJoint(q2, ur5);
    ur5.move_joints(q2_chosen,t/2);
    pause(t);
    pause(0.5);
        q_recent = ur5.get_current_joints();
        g_recent = ur5FwdKin(q_recent);
    error=testError(gt,g_recent);
    fprintf('error of invkin:')
    disp(error);
    %move back
    ur5.move_joints(q2above_chosen,t/2);
    pause(t);
    display("step 5 ");
    
 %% Select the best set of joint

end