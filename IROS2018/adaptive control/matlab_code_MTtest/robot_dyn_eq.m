% function [MM,CC,gg, Jt] = robot_dyn_eq(S, M, J,f_iti, q,q_dot)
function [MM,CC,gg] = robot_dyn_eq(robot,q,q_dot)
n_joints=robot.nDOF;
SS=zeros(n_joints*6,n_joints);
GG=eye(6*n_joints, 6*n_joints);
JJ=zeros(6*n_joints, 6*n_joints);
adV=zeros(6*n_joints, 6*n_joints);
f_=cell(n_joints,1);
% Ad_ft=zeros(6*n_joints, 6*n_joints);

for i=1:n_joints
    SS(6*(i-1)+1:6*i,i)= robot.link(i).screw;
    JJ(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=robot.link(i).J;
    f_{i}=robot.link(i).M*SE3_exp(robot.link(i).screw*q(i));
%     Ad_ft(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=Ad_T(invSE3(f_iti{i}));

    if i>1
        f_ji=f_{i};
        for j=(i-1):-1:1
            GG(6*(i-1)+1:6*i,6*(j-1)+1:6*j)=Ad_T(invSE3(f_ji));
            f_ji=f_{j}*f_ji;
        end
    end
end
V=GG*SS*q_dot;
for i=1:n_joints
    adV(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=ad_V(V(6*(i-1)+1:6*i,1));
end
P0=zeros(6*n_joints,6);
P0(1:6,1:6)=Ad_T(invSE3(f_{1}));


MM=SS'*GG'*JJ*GG*SS;
CC=SS'*GG'*(JJ*GG*adV-adV'*JJ*GG)*SS;
gg=SS'*GG'*JJ*GG*P0*[0;0;0;0;0;9.8];
% Jt=-Ad_ft*GG*SS;
end