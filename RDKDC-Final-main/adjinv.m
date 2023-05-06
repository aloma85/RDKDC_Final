function ans = adjinv(g)
R = g(1:3,1:3);
p = g(1:3,4);
ans = [
    R', -SKEW3(R'*p)*R'; 
    zeros(3,3), R';
    ];
end