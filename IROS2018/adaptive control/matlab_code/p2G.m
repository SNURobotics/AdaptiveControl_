function G = p2G(p)
n_parts = int8(size(p,1)/10);
G = zeros(6,6,n_parts);
for i = 1 : n_parts
    p_cur = p(10*(i-1) + 1:10*i,1);
    G(1,1,i) = p_cur(5);
    G(2,2,i) = p_cur(6);
    G(3,3,i) = p_cur(7);
    G(1,2,i) = p_cur(8);
    G(2,3,i) = p_cur(9);
    G(1,3,i) = p_cur(10);
    G(3,5,i) = p_cur(2);
    G(1,6,i) = p_cur(3);
    G(2,4,i) = p_cur(4);
    G(2,6,i) = -p_cur(2);
    G(3,4,i) = -p_cur(3);
    G(1,5,i) = -p_cur(4);
    G(4,4,i) = p_cur(1);G(5,5,i) = p_cur(1);G(6,6,i) = p_cur(1);
    for j = 2 : 6
        for k =1 : j-1
            G(j,k,i) = G(k,j,i);
        end
    end
end

end