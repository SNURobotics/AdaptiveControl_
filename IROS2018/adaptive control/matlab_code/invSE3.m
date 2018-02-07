function g_inv = invSE3(g)

R = g(1:3,1:3);
p = g(1:3,4);
g_inv = [R.' -R.'*p; zeros(1,3) 1];