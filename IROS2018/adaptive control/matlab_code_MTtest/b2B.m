function B = b2B(b)
n_parts = int8(size(b,1)/10);
B = zeros(4,4,n_parts);
L = zeros(3,3);
for i = 1 : n_parts
    b_cur = b(10*(i-1) + 1:10*i,1);
    B(4,4,i) = b_cur(1);
    B(1:3,4,i) = b_cur(2:4);
    B(4,1:3,i) = b_cur(2:4)';
    L = [b_cur(5), b_cur(8)/2, b_cur(10)/2;
         b_cur(8)/2, b_cur(6), b_cur(9)/2;
         b_cur(10)/2, b_cur(9)/2, b_cur(7)];
    B(1:3,1:3,i) = trace(L)*eye(3,3) - L;
end
end