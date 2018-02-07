function R = RotY(q)

R = [cos(q) 0 sin(q);
    0 1 0;
    -sin(q) 0 cos(q)];