% global g
% g = 9.8;

% robot = wam7robot;
robot = wam7robot_with_load;

robot_0 = wam7robot_0;
color = rand(robot.nDOF,3);
%% plot in zero position
theta = zeros(robot.nDOF,1);theta(1) = pi/2;
[T, Tsave] = forkine(robot,theta);
G = zeros(6,6,robot.nDOF);
for j =1 :robot.nDOF
    G(:,:,j) = robot.link(j).J;
end
plot_inertiatensor(Tsave, G, 0.5, color); hold on;
draw_SE3(eye(4,4));
for i = 1 : robot.nDOF
    draw_SE3(Tsave(:,:,i));
end


%% simluation
global state0_parameter
n_time = 2000;
tspan = linspace(0,60,n_time);
state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
[state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory(0, robot_0);  %% setting inital joint angles and velocities with error zero
state0_parameter = zeros(robot_0.nDOF*10,1); % set of inertial parameters
for i =1 : robot.nDOF
    state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot_0.link(i).J);
end
state0_augmented = [state0_joint;state0_parameter];
[t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
% [t, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);


% state_joint = state_augmented(:,1:robot.nDOF)';
% tic
% for k =1 : size(state_joint,2)
%     theta = state_joint(:,k);
% [T, Tsave] = forkine(robot,theta);
% G = zeros(6,6,robot.nDOF);
% for j =1 :robot.nDOF
%     G(:,:,j) = robot.link(j).J;
% end
% plot_inertiatensor(Tsave, G, 0.1, color);hold on;
% draw_SE3(eye(4,4));
% for i = 1 : robot.nDOF
%     draw_SE3(Tsave(:,:,i));
% end
% drawnow;
% hold off;
% end
% toc


%%
q_desired = zeros(robot.nDOF,n_time);
q_desired_dot = zeros(robot.nDOF,n_time);
q_desired_ddot = zeros(robot.nDOF,n_time);
for j = 1 : n_time
    [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory(t(j), robot);
end
for i = 1 : robot.nDOF
    figure(i); hold on;
    plot(t, state_augmented(:,i));
    plot(t, q_desired(i,:));
end


%%
G = zeros(6,6,robot.nDOF);
for j =1 :robot.nDOF
    G(:,:,j) = robot.link(j).J;
end
for k =1 :2:  size(state_augmented,1)
    theta = state_augmented(k,1:robot.nDOF);
[T, Tsave] = forkine(robot,theta);
G_out = p2G(state_augmented(k,2*robot.nDOF+1:end)');
aa = figure(10); 
set(aa, 'position',[150 150 1200 500]);
a = subplot(1,1,1);
plot_inertiatensor(Tsave, G_out, 0.7, color);hold on;
draw_SE3(eye(4,4));
for i = 1 : robot.nDOF
    draw_SE3(Tsave(:,:,i));
end
axis([-1.0 1.0 -1.0 1.0 -1.0 1.0]);
set(a,'position',[0.1 0.1 0.35,0.8]);
% hold off;
b = subplot(1,2,2);
hold on; ylim([-2 2]);
y = [reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1)];
bar(y);
set(b,'position',[0.55 0.1 0.35,0.8]);
set(b, 'xtick', [1:robot.nDOF], 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
drawnow;
end
G_out = p2G(state_augmented(end,2*robot.nDOF+1:end)');
plot_inertiatensor(Tsave(:,:,end), G_out(:,:,end), 0.7, color(end,:));hold on;
plot_inertiatensor(Tsave(:,:,end), G(:,:,end), 0.7, color(end,:));hold on;