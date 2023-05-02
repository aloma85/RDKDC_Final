function xi = getXi(g)
    R = g(1:3, 1:3);
    p = g(1:3, 4);
    trR = trace(R);
    th = acos((trR - 1) / 2);
    xi = zeros(6, 1);
    
    if th == 0
        w = [0; 0; 0];
        v = p / sqrt(p' * p);
    else
        W = (R - R') / (2 * sin(th));
        w = [W(3, 2); W(1, 3); W(2, 1)];
        A = th * eye(3) + (1 - cos(th)) * W + (th - sin(th)) * W^2;
        v = A \ p;
    end
    
    xi(1:3) = v;
    xi(4:6) = w;

end

