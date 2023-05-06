clc 
clear all
close all
ur5 = ur5_interface();

T = 1e-5; 
gst1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
gst2 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
d = 0.1;
t = 10;
gst1_above = gst1;
gst1_above(3,4) = gst1_above(3,4) + d;
gst2_above = gst2;
gst2_above(3,4) = gst2_above(3,4) + d;

%%  home configuration
ur5 = ur5_interface();

ur5.move_joints(ur5.home,t);
% % q_recent = ur5.get_current_joints();
% % 
% % g0 = ur5FwdKin([0;0;0;0;0;0]);
% % ur5RRcontrol(q_recent,g0,0.1);
% % pause(1);

%% arrive above start position

q_recent = ur5.get_current_joints();
ur5RRcontrol(q_recent,gst1_above,T);
pause(t/4);
display("step1")

%% arrive at start position

q1_above = ur5.get_current_joints();

ur5RRcontrol(q1_above,gst1,T);
pause(t/4);
display("step 2");

%% arrive above target position 2

q1 = ur5.get_current_joints();
ur5RRcontrol(q1,gst2_above,T);
display("step 3");
pause(t/4);

%% arrive at target position 2

q2_above = ur5.get_current_joints();
ur5RRcontrol(q2_above,gst2,T);
pause(t/4);
display("step 4");

%% arrive above target position 2
q2 = ur5.get_current_joints();
ur5.move_joints(q2,gst2_above);
pause(t)

%% come back to home figuration
q_recent = ur5.get_current_joints;
g0 = ur5FwdKin([0;0;0;0;0;0]);
ur5RRcontrol(q_recent,g0);