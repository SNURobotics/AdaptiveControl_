function p = G2p(G)
n_parts = size(G,3);
p = zeros(10*n_parts,1);
for i = 1 : n_parts
    p(10*(i-1) + 1:10*i,1) = [G(4,4,i); G(3,5,i);G(1,6,i);G(2,4,i); G(1,1,i);G(2,2,i);G(3,3,i);G(1,2,i);G(2,3,i);G(1,3,i)];
end

end