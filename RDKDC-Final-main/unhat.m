function a = unhat(aHat)
    wHat = aHat(1:3,1:3);
    v = aHat(1:3,4);
    w = [wHat(3,2),wHat(1,3),wHat(2,1)];
    a = [v;w'];
end
