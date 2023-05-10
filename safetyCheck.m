function result = safetyCheck(q1)

    %security protection
%     l0 = 0.0892;
%     l1 = 0.425;
%     l2 = 0.392;
%     l3 = 0.1093;
%     l4 = 0.09475;
%     l5 = 0.0825;
                
    % To avoid the UR5 touch the table
%     a1=l1*sin(-q1(2));
%     a2=a1+l2*sin(pi+q1(2)+q1(3));
%     a3=a2+l4*sin(q1(2)+q1(3)+q1(4)+3*pi/2);
%     a4=a3-l5*sin(q1(5));
    for i=1:length(q1)
        if q1(i) <= -pi || q1(i) >= pi
            disp(("Joint " + (i-1) + " is reaching its limit."));
            result = 0;
            return;
        end
    end
    gst = ur5FwdKin(q1);
    g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];
    g_T_pen = eye(4); % measure this
    g_baseK_pen = g_baseK_S*gst*g_T_pen;
    % To avoid singularities
    ability1 = manipulability("detjac",ur5BodyJacobian(q1)) ;
    ability2 = manipulability("sigmamin",ur5BodyJacobian(q1)) ;
    ability3 = manipulability("invcond",ur5BodyJacobian(q1)) ;
    if abs(ability1) < 1e-5 ||abs(ability2) < 1e-5||abs(ability3) < 1e-5
        result = 0;
        disp('The ur5 is reaching a singularity!');
    elseif g_baseK_pen(3, 4) < 0
        disp("The ur5 is about to hit the table!");
        result = 0;  
    else
        result = 1;
    end
%     result = 1;

end