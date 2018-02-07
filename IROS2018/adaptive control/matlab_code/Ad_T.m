function AdT=Ad_T(T)
R=T(1:3,1:3);
p=T(1:3,4);
AdT=[[R;skew(p)*R],[zeros(3,3);R]];
end