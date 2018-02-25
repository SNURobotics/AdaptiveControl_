function [Y] = adaptation_regressor_recursive(robot, q, qdot, a, v)
V = zeros(6,robot.nDOF);
W = zeros(6,robot.nDOF);
X = zeros(6,robot.nDOF);
F = zeros(6,robot.nDOF);
% J = zeros(6,6,robot.nDOF);
% J= p2G(parameter);

T = zeros(4,4,robot.nDOF);
Adj_inv_T = zeros(6,6,robot.nDOF);



%% Forward recursion
X0 = [0;0;0;0;0;9.8];
for i = 1 : robot.nDOF
    T(:,:,i) = robot.link(i).M*SE3_exp(robot.link(i).screw*q(i));
    Adj_inv_T(:,:,i) = Adj_inv(T(:,:,i));
    
    if i >1
        V(:,i) = Adj_inv_T(:,:,i)*V(:,i-1) + robot.link(i).screw * qdot(i);
        W(:,i) = Adj_inv_T(:,:,i)*W(:,i-1) + robot.link(i).screw * v(i);
        X(:,i) = Adj_inv_T(:,:,i)*X(:,i-1) + ad_V(V(:,i))*robot.link(i).screw*v(i) + robot.link(i).screw*a(i);
    else
        V(:,i) = robot.link(i).screw*qdot(i);
        W(:,i) = robot.link(i).screw*v(i);
        X(:,i) = Adj_inv_T(:,:,i)*X0 + ad_V(V(:,i))*robot.link(i).screw*v(i) + robot.link(i).screw*a(i);
    end
end

%% Backward recursion
SS=zeros(robot.nDOF*6,robot.nDOF);
GG=eye(6*robot.nDOF, 6*robot.nDOF);
Y_hat = zeros(6*robot.nDOF, 10*robot.nDOF);
% Y = zeros(robot.nDOF, 10*robot.nDOF);

for i=1:robot.nDOF
    SS(6*(i-1)+1:6*i,i)= robot.link(i).screw;
    if i>1
        f_ji=T(:,:,i);
        for j=(i-1):-1:1
            GG(6*(i-1)+1:6*i,6*(j-1)+1:6*j)=Ad_T(invSE3(f_ji));
            f_ji=T(:,:,j)*f_ji;
        end
    end
%     Y_hat(6*(i-1)+1 : 6*i, 10*(i-1)+1 : 10*i) = V_regressor(X(:,i)) - ad_V(V(:,i))'*V_regressor(W(:,i)) + V_regressor(ad_V(V(:,i))*W(:,i));
    Y_hat(6*(i-1)+1 : 6*i, 10*(i-1)+1 : 10*i) = V_regressor(X(:,i)) - ad_V(V(:,i))'*V_regressor(W(:,i));
end

Y = SS'*GG'*Y_hat;



end