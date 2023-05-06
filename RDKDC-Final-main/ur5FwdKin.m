%% ----Question 1. (a)-----%
%  – Purpose: Compute the forward kinematic map of the UR5. All necessary parameters 
% (e.g. the base twists, gst0, etc) should be defined inside the function.

% – Inputs: q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4, θ5, θ6]T where θn is the angle of joint n for n = 1,··· ,6. Be careful of sign convention!
% – Output: gst: end effector pose, gst (4 × 4 matrix)

function [gst,g,Tw] = ur5FwdKin(q)
    % Define in  m
    l0 = 0.0892;
    l1 = 0.425;
    l2 = 0.392;
    l3 = 0.1093;
    l4 = 0.09475;
    l5 = 0.0825;
    
    % w
    ex = [1; 0; 0];
    ey = [0; 1; 0];
    ez = [0; 0; 1];
    w = [ez, ey, ey, ey, -ez, ey];
    
    % q
    q1 = [0; 0; 0];
    q2 = [0; 0; l0];
    q3 = [l1; 0; l0];
    q4 = [l1+l2; 0; l0];
    q5 = [l1+l2; l3; l0-l4];
    q6 = [l1 + l2; l3 + l5; l0 - l4];

    % Calculating twists 
    Tw1 = [cross(q1, w(:,1)); w(:,1) ];
    Tw2 = [cross(q2, w(:,2)); w(:,2) ];
    Tw3 = [cross(q3, w(:,3)); w(:,3) ];
    Tw4 = [cross(q4, w(:,4)); w(:,4) ];
    Tw5 = [cross(q5, w(:,5)); w(:,5) ];
    Tw6 = [cross(q6, w(:,6)); w(:,6) ];
    Tw = [Tw1,Tw2,Tw3,Tw4,Tw5,Tw6];
   
    
%     gst0 = [[0 1 0; 1 0 0; 0 0 -1] [ l1+l2; l3 + l5; l0 - l4]; 0 0 0 1];
    gst0 = [ROTX(-pi/2)* ROTZ(pi) [l1+l2; l3 + l5; l0 - l4]; 0 0 0 1];
    
    g1 = expm(wedge(Tw1) * q(1));
    g2 = expm(wedge(Tw2) * q(2));
    g3 = expm(wedge(Tw3) * q(3));
    g4 = expm(wedge(Tw4) * q(4));
    g5 = expm(wedge(Tw5) * q(5));
    g6 = expm(wedge(Tw6) * q(6));


    g(:,:,1) = eye(4);
    g(:,:,2) = g1;
    g(:,:,3) = g1*g2;
    g(:,:,4) = g1*g2*g3;
    g(:,:,5) = g1*g2*g3*g4;
    g(:,:,6) = g1*g2*g3*g4*g5;


    
    gst = g1 * g2 * g3 * g4 * g5 * g6 * gst0;



function Ans = wedge(Tw)
% Calculates the wedge of provided Twist(X)

A = SKEW3(Tw(4:6));
B = Tw(1:3);


Ans = [A B; 0 0 0 0];

function skewM = SKEW3 (Vs)
    skewM = [0,-Vs(3), Vs(2); Vs(3), 0, -Vs(1); -Vs(2), Vs(1), 0];  
end

end


end
