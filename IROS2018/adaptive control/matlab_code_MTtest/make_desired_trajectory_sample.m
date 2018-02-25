function [q_desired, q_desired_dot, q_desired_ddot] = make_desired_trajectory_sample(t, A, robot)
global PeriodDesiredTraj

q_desired = zeros(robot.nDOF,1);
q_desired_dot = zeros(robot.nDOF,1);
q_desired_ddot = zeros(robot.nDOF,1);


for i = 1 : robot.nDOF
    for k = 1
        w = k*2*pi/PeriodDesiredTraj;
        q_desired(i) = q_desired(i) + A * (cos(w*t)-1);
        q_desired_dot(i) = q_desired_dot(i) - A * w * sin(w*t);
        q_desired_ddot(i) = q_desired_ddot(i) - A * w^2 * cos(w*t);
    end
end



% a = zeros(robot.nDOF, 5); b = zeros(robot.nDOF, 5); q0 = zeros(robot.nDOF,1);
% w_f = 0.1 * pi;
% a = [  0.05, -0.29, 0.48, 0.55, 0.65;
%  0.03, 0.29, -0.23, 0.32, 0.82;
%  -0.07, 0.40, 0.45, 0.40, -0.03;
%  0.14, -0.35, 0.15, 0.11, 0.93;
%  0.21, 0.35, 0.16, -0.02, 0.03;
%  -0.11, 0.28, 0.36, -0.06, 0.33;
%  -0.01, 0.24, 0.37, -0.45, 0.75];
% b = [ 0.19, -0.40, -0.18, 0.63, -0.46 ;
%  0.09, -0.08, 0.05, -0.02, 0.65 ;
%  -0.49, 0.32, -0.26, -0.63, 0.06 ;
%  -0.14, 0.06, -0.13, -0.14, -0.03 ;
%  -0.51, 0.14, 0.37, -0.15, -0.17 ;
%  0.13, 0.07, 0.67, -0.15, -0.22 ;
%  0.24, 0.24, -0.22, -0.50, -0.52 ];
% q0 = [-0.29; 0.11; -0.02; 1.67; -2.41; 0.20; 0.58];
% 
% for k = 1 : robot.nDOF
%     for l = 1 : 5
%         q_desired(k) = q_desired(k) + (a(k,l)/(w_f*l))*sin(w_f*l*t) - (b(k,l)/(w_f*l))*cos(w_f*l*t);
%         q_desired_dot(k) = q_desired_dot(k) + (a(k,l))*cos(w_f*l*t) + (b(k,l))*sin(w_f*l*t);
%         q_desired_ddot(k) = q_desired_ddot(k) - (a(k,l)*(w_f*l))*sin(w_f*l*t) + (b(k,l)*(w_f*l))*cos(w_f*l*t);
%     end
%     q_desired(k) = q_desired(k) + q0(k);
% end






end