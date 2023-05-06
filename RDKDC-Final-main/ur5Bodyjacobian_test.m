q=[pi/2,pi/3,pi/4,pi/5,pi/2,pi/3]';
e=eye(6);
k=pi/100;
S=zeros(6);
gst=ur5FwdKin(q);
for i=1:6
        p=(ur5FwdKin(q+e(:,i)*k)-ur5FwdKin(q-e(:,i)*k))/(2*k);
        s(:,i)=getXi(inv(gst)*p);
end

j=ur5BodyJacobian(q);
err=norm(s-j)

