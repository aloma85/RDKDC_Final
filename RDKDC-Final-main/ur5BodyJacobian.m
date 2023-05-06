function JB = ur5BodyJacobian(q)
   
    [gst,g,Tw] = ur5FwdKin(q); 
    JS = zeros(6,6);
    JB = zeros(6,6);


    for i = 1:6 
        JS(:,i) = adj(g(:,:,i))*Tw(:,i);
    end

    JB = adjinv(gst)*JS;
end


