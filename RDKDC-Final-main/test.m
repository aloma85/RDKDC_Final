ur5 = ur5_interface();
% q=[0,0,0,0,0,0]';
% ur5.move_joints(q,5);
K = 0.7;
qk = [1.1310; -0.6283; 1.0053; 0.5027; -0.1885; 0.4398];
% gdesired = ur5FwdKin(qk+[0.5;0;0;0;0;0]);
gdesired = ur5FwdKin(qk);
ur5RRcontrol(gdesired, K, ur5);