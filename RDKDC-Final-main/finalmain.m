clear;

clc;

ur5=ur5_interface();
%start position:
 start=input("Please press enter to record the start position");
 qs=ur5.get_current_joints();
%qs=[0.6283,   -0.8168,    0.8168,   -1.5708,   -1.2566,   -0.6283].';
%target position:
 target=input("Please press enter to record the target position");
 qt=ur5.get_current_joints();
%qt=[1.7562,   -0.8200,    0.8200,   -1.3195,   -1.6336,   -0.6300]';

%choose mode
mode=input("Please choose mode:1 represents Invkin, 2 represents RRInv, 3 represents RRTran");
while (mode~=1 && mode~=2 && mode~=3)
    mode=input("PLease enter 1,2 or 3:1 represents Invkin, 2 represents RRInv, 3 represents RRTran");
end
if mode==1
    Invkin(qs,qt,ur5);
    
elseif mode==2
    
        ur5RRInvJ(qs,qt,ur5)
elseif mode==3
        TransposeJacobian(qs,qt,ur5)
end
