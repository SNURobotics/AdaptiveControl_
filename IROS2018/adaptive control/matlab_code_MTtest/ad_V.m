function adV=ad_V(V)
w=V(1:3,1);
v=V(4:6,1);
adV=[[skew(w);skew(v)],[zeros(3,3);skew(w)]];
end