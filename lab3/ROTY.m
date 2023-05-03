function R = ROTY(pitch)
    R = [cos(pitch), 0, sin(pitch);
        0, 1, 0;
        -sin(pitch), 0, cos(pitch)];
end