function S = G2S(G)
n_parts = size(G,3);
S = zeros(4,4,n_parts);
for i = 1 : n_parts
    S(1:3,1:3,i) = 0.5*trace(G(1:3,1:3,i))*eye(3,3)-G(1:3,1:3,i);
    S(1:3,4,i) = skew(G(1:3,4:6,i));
    S(4,1:3,i) = skew(G(1:3,4:6,i))';
    S(4,4,i) = G(4,4,i);
end


end