function [A,B,B_J, b] = regression_mat(q, dotq, dotdotq, tau, robot)

global g
DOF = robot.nDOF;
checkDimension(q, robot);
checkDimension(dotq, robot);
checkDimension(dotdotq, robot);

if nargin < 6
    Fext = [];
end
% tic;
% fixed base
V0 = zeros(6,1);
% assume gravity is acting in z-axis of global frame
dotV0 = [zeros(5,1); g];

% tau : joint torque
% tau = zeros(DOF,1);

% F : Generalized force from {i-1} th link to {i} th link
F = zeros(6,DOF+1);

% T : SE(3) from {i-1} th link frame to {i} th link frame
T = zeros(4,4,DOF);
Adj_inv_T = zeros(6,6,DOF);
% V : Generalized velocity of {i} th coordinate frame
V = zeros(6,DOF);
dotV = zeros(6,DOF);
adj_V = zeros(6,6,DOF);
% Forward recursion
% toc;
T(:,:,1) = robot.base*robot.link(1).M*LargeSE3(robot.link(1).screw(1:3)*q(1), robot.link(1).screw(4:6)*q(1));
Adj_inv_T(:,:,1) = Adj_inv(T(:,:,1));
V(:,1) = Adj_inv_T(:,:,1)*V0 + robot.link(1).screw*dotq(1);
adj_V(:,:,1) = adjoint(V(:,1));
dotV(:,1) = robot.link(1).screw*dotdotq(1) + Adj_inv_T(:,:,1)*dotV0 + adj_V(:,:,1)*robot.link(1).screw*dotq(1);

for i=2:DOF
    T(:,:,i) = robot.link(i).M*LargeSE3(robot.link(i).screw(1:3)*q(i), robot.link(i).screw(4:6)*q(i));
    Adj_inv_T(:,:,i) = Adj_inv(T(:,:,i));
    Sdotq = robot.link(i).screw*dotq(i);
    V(:,i) = Adj_inv_T(:,:,i)*V(:,i-1) + Sdotq;
    adj_V(:,:,i) = adjoint(V(:,i));
    dotV(:,i) = robot.link(i).screw*dotdotq(i) + Adj_inv_T(:,:,i)*dotV(:,i-1) + adj_V(:,:,i)*Sdotq;
end
B_J = zeros(6,6,robot.nDOF,robot.nDOF);
B = zeros(4,4,robot.nDOF,robot.nDOF);
A = zeros(robot.nDOF, 10*robot.nDOF);
b = zeros(robot.nDOF,1);
% Backward recursion (consider joint friction)
if isempty(Fext)
%     F(:,DOF) = robot.link(DOF).J*dotV(:,DOF) - adj_V(:,:,DOF).'*robot.link(DOF).J*V(:,DOF);
%     tau(DOF) = robot.link(DOF).screw.'*F(:,DOF) + robot.joints.kc(DOF)*sign(dotq(DOF)) + robot.joints.kv(DOF)*dotq(DOF)+robot.joints.ko(DOF);
    B_J(:,:,DOF,DOF) = dotV(:,DOF)*robot.link(DOF).screw'-V(:,DOF)*robot.link(DOF).screw'*adj_V(:,:,DOF)';
    b(DOF) = tau(DOF) - robot.joints.kc(DOF)*sign(dotq(DOF)) - robot.joints.kv(DOF)*dotq(DOF) - robot.joints.ko(DOF); 
    for i = DOF-1:-1:1
        B_J(:,:,i,i) = dotV(:,i)*robot.link(i).screw'-V(:,i)*robot.link(i).screw'*adj_V(:,:,i)';
        Adj_inv_T_j = eye(6,6);
        for j = i+1 : DOF
            Adj_inv_T_j = Adj_inv_T(:,:,j)*Adj_inv_T_j;
            B_J(:,:,j,i) = dotV(:,j)*robot.link(i).screw'*Adj_inv_T_j'-V(:,j)*robot.link(i).screw'*Adj_inv_T_j'*adj_V(:,:,j)';
        end
%         F(:,i) = Adj_inv_T(:,:,i+1).'*F(:,i+1) + robot.link(i).J*dotV(:,i) - adj_V(:,:,i).'*robot.link(i).J*V(:,i);
%         tau(i) = robot.link(i).screw.'*F(:,i) + robot.joints.kc(i)*sign(dotq(i)) + robot.joints.kv(i)*dotq(i) + robot.joints.ko(i);
        b(i) = tau(i) - robot.joints.kc(i)*sign(dotq(i)) - robot.joints.kv(i)*dotq(i) - robot.joints.ko(i);
    end
else
    F(:,DOF) = robot.link(DOF).J*dotV(:,DOF) - adj_V(:,:,DOF).'*robot.link(DOF).J*V(:,DOF) - Adj_inv(robot.link(DOF).Ttool).'*Fext(:,DOF);
%     tau(DOF) = robot.link(DOF).screw.'*F(:,DOF) + robot.joints.kc(DOF)*sign(dotq(DOF)) + robot.joints.kv(DOF)*dotq(DOF);
    for i = DOF-1:-1:1
        F(:,i) = Adj_inv_T(:,:,i+1).'*F(:,i+1) + robot.link(i).J*dotV(:,i) - adj_V(:,:,i).'*robot.link(i).J*V(:,i) - Adj_inv(robot.link(i).Ttool).'*Fext(:,i);
%         tau(i) = robot.link(i).screw.'*F(:,i) + robot.joints.kc(i)*sign(dotq(i)) + robot.joints.kv(i)*dotq(i);
    end
end

for i = 1 : robot.nDOF
    for j =1 : robot.nDOF
        B_J(:,:,j,i) = (B_J(:,:,j,i) + B_J(:,:,j,i)')/2;
        B(:,:,j,i) = [trace(B_J(1:3,1:3,j,i))*eye(3,3)-B_J(1:3,1:3,j,i), skew(B_J(1:3,4:6,j,i)-B_J(1:3,4:6,j,i)');skew(B_J(1:3,4:6,j,i)-B_J(1:3,4:6,j,i)')',trace(B_J(4:6,4:6,j,i))];
        A(i,10*(j-1)+1:10*j) = [B(4,4,j,i), B(4,1:3,j,i)*2,B(1,1,j,i),B(2,2,j,i),B(3,3,j,i), B(1,2,j,i)*2,B(2,3,j,i)*2,B(3,1,j,i)*2];
    end
end


end