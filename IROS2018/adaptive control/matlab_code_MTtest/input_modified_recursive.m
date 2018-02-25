function [tau] = input_modified_recursive(robot, q, qdot, a, v, parameter)
V = zeros(6,robot.nDOF);
W = zeros(6,robot.nDOF);
X = zeros(6,robot.nDOF);
F = zeros(6,robot.nDOF);
% J = zeros(6,6,robot.nDOF);
J= p2G(parameter);
tau = zeros(robot.nDOF,1);

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
for i = robot.nDOF: -1 : 1
    if i < robot.nDOF
%         F(:,i) = Adj_inv_T(:,:,i+1)'*F(:,i+1) + J(:,:,i) * X(:,i) - ad_V(V(:,i))'*J(:,:,i)*W(:,i) + J(:,:,i)*ad_V(V(:,i))*W(:,i);
        F(:,i) = Adj_inv_T(:,:,i+1)'*F(:,i+1) + J(:,:,i) * X(:,i) - ad_V(V(:,i))'*J(:,:,i)*W(:,i);
    else
%         F(:,i) = J(:,:,i) * X(:,i) - ad_V(V(:,i))'*J(:,:,i)*W(:,i) + J(:,:,i)*ad_V(V(:,i))*W(:,i);
        F(:,i) = J(:,:,i) * X(:,i) - ad_V(V(:,i))'*J(:,:,i)*W(:,i);
    end
    tau(i) = robot.link(i).screw'*F(:,i);
end



end