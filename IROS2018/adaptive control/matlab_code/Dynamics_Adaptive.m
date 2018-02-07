function [dot_state_augmented] = Dynamics_Adaptive(t, state_augmented, robot)
global state0_parameter
dot_state_joint = zeros(robot.nDOF*2,1);
dot_state_parameter = zeros(robot.nDOF*10,1);

input_torque = zeros(robot.nDOF,1);

state_joint_q = state_augmented(1:robot.nDOF,1);
state_joint_qdot = state_augmented(robot.nDOF+1:robot.nDOF*2,1);
state_parameter = state_augmented(robot.nDOF*2+1:end,1);

[MM,CC,gg] = robot_dyn_eq(robot,state_joint_q,state_joint_qdot);


%% Define desired trajectory
%%%% 트레젝토리 바꾸기!!!
q_desired = zeros(robot.nDOF,1);
q_desired_dot = zeros(robot.nDOF,1);
q_desired_ddot = zeros(robot.nDOF,1);
[q_desired, q_desired_dot, q_desired_ddot] = make_desired_trajectory(t, robot);

%%%%%%%%%%%%%%%%   passivity based control   %%%%%%%%%%%%%%%%
%% Define control law
lambda = 30.0;
Lambda_mat = lambda * eye(robot.nDOF,robot.nDOF);
a_input = q_desired_ddot - Lambda_mat * (state_joint_qdot - q_desired_dot);
v_input = q_desired_dot - Lambda_mat * (state_joint_q - q_desired);
r_input = state_joint_qdot - v_input;
input_torque = input_modified_recursive(robot, state_joint_q, state_joint_qdot, a_input, v_input, state_parameter);
K = lambda * Mass_matrix(robot,state_parameter, state_joint_q);
input_torque = input_torque - K * r_input;

%% Define adaptation law
Gamma_inv = 100.0 ; % natural
% Gamma_inv = 0.00001; % euclidean
v_adapt = v_input;
a_adapt = a_input - lambda * r_input;
Y = adaptation_regressor_recursive(robot, state_joint_q, state_joint_qdot, a_adapt, v_adapt);
b = Y'*r_input;
%%% only adapt last link %%%
% b(1:10*(robot.nDOF-1),1) = zeros(10*(robot.nDOF-1),1);

%%% natural adaptation %%%
B = b2B(b);
P_hat = G2S(p2G(state_parameter));
P_init = G2S(p2G(state0_parameter));
for i = 1 : robot.nDOF
%     b(10*(i-1)+1:10*i,1) = G2p(S2G(P_hat(:,:,i)*B(:,:,i)*P_hat(:,:,i)));
%%% only adapt last link %%%%%%
    if i == robot.nDOF
        b(10*(i-1)+1:10*i,1) = G2p(S2G(P_hat(:,:,i)*B(:,:,i)*P_hat(:,:,i))); 
    else
        b(10*(i-1)+1:10*i,1) = zeros(10,1);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     b(10*(i-1)+1:10*i,1) = G2p(S2G(P_init(:,:,i)*B(:,:,i)*P_init(:,:,i))); % constant P_hat with initial value
end

%% Define Dynamics update

dot_state_joint(1:robot.nDOF,1) = state_joint_qdot;  %qdot 
dot_state_joint(robot.nDOF+1:end,1) = pinv(MM)* ( -CC * state_joint_qdot  -gg  + input_torque); % qddot
dot_state_parameter = - Gamma_inv * b;

dot_state_augmented = [dot_state_joint;dot_state_parameter];
end