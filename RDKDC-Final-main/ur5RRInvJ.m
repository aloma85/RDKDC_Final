function ur5RRInvJ(qs,qt,ur5)

    gst1 = ur5FwdKin(qs);
    gst2 = ur5FwdKin(qt);

    d = 0.1;
    t = 2;
%     error=1e-2;
    gst1_above = gst1;
    gst1_above(3,4) = gst1_above(3,4) + d;
    gst2_above = gst2;
    gst2_above(3,4) = gst2_above(3,4) + d;
    
    %  home configuration   
    ur5.move_joints([0.1079,-1.9040,1.0062,-1.4970,-0.2995,0.0007]',6)
    pause(11)   
%    while 1
%         q_recent = ur5.get_current_joints();
%         if norm(q_recent-[0.1079,-1.9040,1.0062,-1.4970,-0.2995,0.0007]')<error
%             break
%         end
%         disp("``````")
%         disp(norm(q_recent-[0.1079,-1.9040,1.0062,-1.4970,-0.2995,0.0007]'))
%         pause(2)
%    end
   disp("back to home");
   

    % arrive above start position
    
    ur5RRcontrol(gst1_above,2.5,ur5);
    pause(t/2);
%     while 1
        q_recent = ur5.get_current_joints();
        g_recent = ur5FwdKin(q_recent);
%         if norm(g_recent-gst1_above) < error
%             break
%         end
%         pause(2)
%         disp("``````")
%     end
    display("step1");
    error=testError(gst1,g_recent);
    fprintf('error of ur5RRcontrol:')
    disp(error);
    
    % arrive at start position 
    ur5RRcontrol(gst1,0.05,ur5);
    
%     while 1
%         q_recent = ur5.get_current_joints();
%         g_recent = ur5FwdKin(q_recent);
%         if norm(g_recent(1:3,4)-gst1(1:3,4)) < 2*error
%             break
%         end
%         pause(2)
%         disp("``````")
%         disp(norm(g_recent(1:3,4)-gst1(1:3,4)));
%      end
    
    display("step 2");  
    pause(t)

    
   

    
    ur5RRcontrol(gst1_above,0.02,ur5);
    
%     while 1
%         g_recent = ur5FwdKin(ur5.get_current_joints());
%         if norm(g_recent-gst1_above)<2*error
%             break
%         end
%         pause(5)
%        disp("``````")
%     end
%     
    display("step 3")
    pause(t)

    
    % arrive above target position 2
  
    ur5RRcontrol(gst2_above,2,ur5);

%     while 1
%         g_recent = ur5FwdKin(ur5.get_current_joints());
%         if norm(g_recent-gst2_above)<2*error
%             break
%         end
%         pause(2)
%         disp("``````")
%     end
    display("step 4");
    pause(t)
    
    % arrive at target position 2
      
   
    ur5RRcontrol(gst2,0.05,ur5);
%     while 1
%         g_recent = ur5FwdKin(ur5.get_current_joints());
%         if norm(g_recent-gst2)<2*error
%             break
%         end
%         pause(2)
%         disp("``````")
%     end
        q_recent = ur5.get_current_joints();
        g_recent = ur5FwdKin(q_recent);    
    error=testError(gst2,g_recent);
    fprintf('error of ur5RRcontrol:')
    disp(error);
    display("step 5");
    pause(t)
    
    
    % arrive above target position 2


    ur5RRcontrol(gst2_above,0.01,ur5);
    pause(t/2)
    

end