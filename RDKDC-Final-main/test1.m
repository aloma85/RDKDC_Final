qk = [1.1310; -0.6283; 1.0053; 0.5027; -0.1885; 0.4398];
gdesired=ur5FwdKin(qk+[0.5;0.1;0.2;0.3;0.2;0.1]);
K=0.7;
ur5=ur5_interface();
ur5.move_joints(qk,5);

Tstep = 0.01;
    i=0;
    q = qk;

%     gd_inv=inv(gdesired);

    desiredFrame = tf_frame('base_link','desiredFrame',eye(4));
    desiredFrame.move_frame('base_link',gdesired);

    while(1)
        gst = ur5FwdKin(q);
        g_error = inv(gdesired)*gst;
        s = getXi(g_error);

        q=q-K*Tstep*inv(ur5BodyJacobian(q))*s;

        % check singulariries; if failure, return -1
        if abs(det(ur5BodyJacobian(q))) < 0.001
            finalerr = -1;
            disp('here is a sigularity')
            return
        end

        %get vk and wk 
        R_error = g_error(1:3,1:3);
        P_error = g_error(1:3,4);
        trace_R_error = R_error(1,1) + R_error(2,2) +R_error(3,3);
        theta= acos((trace_R_error-1)/2);
        p = sqrt((P_error(1))^2+(P_error(3))^2+(P_error(3))^2);

        %define the threshold (5cm, 15degree)
        if theta <= 1*pi/180 && p <= 0.001
            finalerr = p*100;
            disp('convengence achieved')
            return
        end
        ur5.move_joints(q,1);
        pause(Tstep)  
        i=i+1

        if i>100
            Tstep=0.005;
        end
        
        if i>500
            Tstep=0.001;
        end
        
        if i>1000
            Tstep=0.0005;
        end

     end






