    function bestQ = selectJoint(q, ur5)
    disp("selecting joints")
%      bestQ = q(:,1);
        
        curq = ur5.get_current_joints();
        for n = 1: 8
            q1 = q(:, n);
            % check of the body jacobian is sigular
            JB = ur5BodyJacobian(q1);
            maniDet = abs(manipulability("detjac", JB));
            mamaniSig = abs(manipulability("sigmamin", JB));
            maniInv = abs(manipulability("invcond", JB));
%             [gst,g,Tw] = ur5FwdKin(q1);
       
%             %get the z axis of these joints
%             g3z = g(3,4,3)
%             g4z = g(3,4,4)
%             g5z = g(3,4,5)
%             g6z = g(3,4,6)
%             if g3z <= 0 || g4z <= 0 ||g5z <= 0 ||g6z <= 0
%                 breaks
%             end
            diff = 3.4028e+38;
            qShoulder = q1(2,:);
            qElbow = q1(3,:);
            q4=q1(4,:);
            q5=q1(5,:);
            
            l0 = 0.0892;
            l1 = 0.425;
            l2 = 0.392;
            l3 = 0.1093;
            l4 = 0.09475;
            l5 = 0.0825;
            

%           % To avoid the UR5 touch the table
            a1=l1*sin(-qShoulder);
            a2=a1+l2*sin(pi+qShoulder+qElbow);
            a3=a2+l4*sin(qShoulder+qElbow+q4+3*pi/2);
            a4=a3-l5*sin(q5);
            if a1<0.01||a2<0.01||a3<0.01||a4<0.01
                disp("check if hit table")
%                 disp('wrong')
%                 ur5.move_joints(q1,5);
%                 disp(a1)
%                 disp(a2)
%                 disp(a3)
%                 disp(q1)
%                 pause(10)
                continue;
            end
%             if -180 < qShoudler && qShoudler < -5 &&  qElbow > 10 && qElbow < 150
    
%                 display("TOUCH TABLE")
%             end
%             
     
            if maniDet > 0.0001 && norm(curq - q1) < diff 
                 diff = norm(curq - q1);
                 display("select")
                 bestQ = q1
           
            end
              
        end
    end
    
%     
%         0.6283
%    -0.8168
%     0.8168
%    -1.5708
%    -1.2566
%    -0.6283

% 
%     1.7562
%    -0.8200
%     0.8200
%    -1.3195
%    -1.6336
%    -0.6300
% 
%     
