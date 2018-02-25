function G = S2G(S)
n_parts = size(S,3);
G = zeros(6,6,n_parts);

for i = 1 : n_parts
    G(1:3,1:3,i) = trace(S(1:3,1:3,i))*eye(3,3) - S(1:3,1:3,i);
    G(1:3,4:6,i) = skew(S(1:3,4,i));
    G(4:6,1:3,i) = skew(S(1:3,4,i))';
    G(4:6,4:6,i) = S(4,4,i)*eye(3,3);
end

end