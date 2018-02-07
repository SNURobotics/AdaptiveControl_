function MM = Mass_matrix(robot,parameter, q)
n_joints=size(q,1);
SS=zeros(n_joints*6,n_joints);
GG=eye(6*n_joints, 6*n_joints);
JJ=eye(6*n_joints, 6*n_joints);
f_=cell(n_joints,1);
J= p2G(parameter);

for i=1:n_joints
    SS(6*(i-1)+1:6*i,i)= robot.link(i).screw;
    JJ(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=J(:,:,i);
    f_{i}=robot.link(i).M*SE3_exp(robot.link(i).screw*q(i));
    if i>1
        f_ji=f_{i};
        for j=(i-1):-1:1
            GG(6*(i-1)+1:6*i,6*(j-1)+1:6*j)=Ad_T(invSE3(f_ji));
            f_ji=f_{j}*f_ji;
        end
    end
end

MM=SS'*GG'*JJ*GG*SS;
end