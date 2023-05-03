function JB = ur5BodyJacobian(q)

    [gst, Twist] = ur5FwdKin(q);
    
    JS = zeros(6, 6);
    adj_g = cell(1, 6);
    prodexp = eye(4);
    JS(:, 1) = Twist(:, 1);
    for i = 1:5
        ex = expm(q(i) * [SKEW3(Twist(4:6, i)), Twist(1:3, i); 0, 0, 0, 0]);
        prodexp = prodexp * ex;
        adj_g{i} = adj(prodexp);
        JS(:, i+1) = adj_g{i} * Twist(:, i+1);
    end

    JB = adj(inv(gst)) * JS;
end
 
function Ad = adj(g)
    R = g(1:3, 1:3);
    phat = SKEW3(g(1:3, 4));
    Ad = [R, phat*R; zeros(3, 3), R];
end
