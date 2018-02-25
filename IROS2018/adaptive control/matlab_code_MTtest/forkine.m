function [T, Tsave] = forkine(robot,q)
% Forward kinematics code from robot information

DOF = robot.nDOF;

T = robot.base;
Tsave = zeros(4,4,DOF);
for i = 1:DOF
    T = T*robot.link(i).M*LargeSE3(robot.link(i).screw(1:3)*q(i), robot.link(i).screw(4:6)*q(i));
    Tsave(:,:,i) = T;
end
T = T*robot.Tendeffector;
end
