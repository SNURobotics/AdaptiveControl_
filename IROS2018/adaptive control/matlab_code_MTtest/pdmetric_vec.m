function metric = pdmetric_vec(P)
n_parts = size(P,3);
metric = zeros(10*n_parts,10*n_parts);
E = zeros(4,4,10);
E(4,4,1) = 1;
E(1,4,2) = 1; E(4,1,2) = 1;
E(2,4,3) = 1; E(4,2,3) = 1;
E(3,4,4) = 1; E(4,3,4) = 1;
E(1,1,5) = 1;
E(2,2,6) = 1;
E(3,3,7) = 1;
E(1,2,8) = 1; E(2,1,8) = 1;
E(2,3,9) = 1; E(3,2,9) = 1;
E(3,1,10) = 1; E(1,3,10) = 1;
for i =1 : n_parts
    for j =1 : 10
        for k = 1: 10 
            metric(10*(i-1)+j,10*(i-1)+k) = trace(pinv(P(:,:,i)) * E(:,:,j) * pinv(P(:,:,i)) * E(:,:,k) ); 
        end
    end
end
end