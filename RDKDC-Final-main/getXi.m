function xi=getXi(g)
R=g(1:3,1:3);
p=g(1:3,4);
theta=acos((trace(R)-1)/2);
if theta==0
    w=[0;0;0];
    v=p/sqrt(p'*p);
else 
    W=R-R';
    W=W/(2*sin(theta));
    w(1,1)=W(3,2);
    w(2,1)=W(1,3);
    w(3,1)=W(2,1);
    v=(theta*eye(3)+(1-cos(theta))*W+(theta-sin(theta))*W^2)\p;
end
xi(1:3,1)=v;
xi(4:6,1)=w;
