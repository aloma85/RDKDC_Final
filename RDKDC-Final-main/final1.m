lc 
clear all
close all
ur5 = ur5_interface();

gst1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
gst2 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
d = 0.1;
t = 10;
gst1_above = gst1;
gst1_above(3,4) = gst1_above(3,4) + d;
gst2_above = gst2;
gst2_above(3,4) = gst2_above(3,4) + d;

% step 0 home configuration
ur5.move_joints(ur5.home,t);
pause(t);

% step 1
q1_above = ur5InvKin(gst1_above);
q1_above_chosen = q1_above(:,1);
ur5.move_joints(q1_above_chosen,t);
pause(t/4);
display("step1")

% step 2
q1 = ur5InvKin(gst1);
q1_chosen = q1(:,1);
ur5.move_joints(q1_chosen,t);
pause(t/4);
display("step 2");

% step 3
ur5.move_joints(q1_above_chosen,t);
pause(t/4);
ur5.move_joints(ur5.home,t);
pause(t/4);
display("step 3");

% step 4
q2_above = ur5InvKin(gst2_above);
q2_above_chosen = q2_above(:,1);
ur5.move_joints(q2_above_chosen,t/2);
pause(t/4);
display("step 4 ");

% step 5
q2 = ur5InvKin(gst2);
q2_chosen = q2(:,1);
ur5.move_joints(q2_chosen,t/2);
ur5.move_joints(q2_above_chosen,t/2);
display("step 5");



    