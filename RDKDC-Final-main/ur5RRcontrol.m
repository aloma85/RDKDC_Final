function [finalerr] =ur5RRcontrol(gdesired, K, ur5)
disp("~~~~~~~~ur5RRcontrol is working~~~~~~~~~~~")
    % initialize Tstep
    Tstep = 0.1;
    t=3;
%     q = [1.1310; -0.6283; 1.0053; 0.5027; -0.1885; 0.4398];

    
    q=ur5.get_current_joints();
    gd_inv = pinv(gdesired);

    %set the frame of gdesired
    desiredFrame = tf_frame('base_link','desiredFrame',eye(4));
    desiredFrame.move_frame('base_link',gdesired);
    pause(t/2);

    % begin the loop to simulate the track
    while 1
        gst = ur5FwdKin(q);
        g_error = gd_inv * gst;
        twist = getXi(g_error);
        
        %calculate the intended angular for this iteration
        dq = K*Tstep*pinv(ur5BodyJacobian(q))*twist;
        

        if max(dq)/t>0.4*pi*ur5.speed_limit
            disp("````````reduce speed````````")
            dq=0.2*dq/(max(dq)/(t*pi*ur5.speed_limit));
             %dq(i)=0.9*sign(dq(i))*pi*ur5.speed_limit*t;
        end
         %security protection
            l0 = 0.0892;
            l1 = 0.425;
            l2 = 0.392;
            l3 = 0.1093;
            l4 = 0.09475;
            l5 = 0.0825;
                        
q1=q-dq;
%             % To avoid the UR5 touch the table
            a1=l1*sin(-q1(2));
            a2=a1+l2*sin(pi+q1(2)+q1(3));
            a3=a2+l4*sin(q1(2)+q1(3)+q1(4)+3*pi/2);
            a4=a3-l5*sin(q1(5));
            if a1<0.01||a2<0.01||a3<0.01||a4<0.01
%                 disp('wrong')
%                 ur5.move_joints(q1,5);
%                 disp(a1)
%                 disp(a2)
%                 disp(a3)
%                 disp(q1)
%                 pause(10)
                dq=dq/10;
                disp("adjusting the location")
                continue;
            end
        
        q=q-dq;
        

        % check singulariries; if failure, return -1
        ability1 = manipulability("detjac",ur5BodyJacobian(q)) ;
        ability2 = manipulability("sigmamin",ur5BodyJacobian(q)) ;
        ability3 = manipulability("invcond",ur5BodyJacobian(q)) ;
        if abs(ability1) < 1e-5 ||abs(ability2) < 1e-5||abs(ability3) < 1e-5
            finalerr = -1;
            disp('here is a sigularity')
            return
        end
        
        %calculate the error of angular and position
        R_error = g_error(1:3,1:3);
        P_error = g_error(1:3,4);
        trace_R_error = R_error(1,1) + R_error(2,2) +R_error(3,3);
        theta= acos((trace_R_error-1)/2);
        p = sqrt((P_error(1))^2+(P_error(2))^2+(P_error(3))^2);
    

        %define the threshold (5cm, 15degree)
        if theta <= 2*pi/180 && p <= 0.02
            finalerr = theta*100;
            disp('convengence achieved')
            return
        end

        %move the ur5
        ur5.move_joints(q,t);
        pause(1.05*t)
   
 display(p)
        %calculate Tstep for next iteration
        if theta <= 20*pi/180 || p <= 0.3
            Tstep=p/3;
            if Tstep<0.02
                t=1.5;
            else
                t=3;
            end
        else
            Tstep=0.1;
        end
            display(Tstep)
    end
   
end





