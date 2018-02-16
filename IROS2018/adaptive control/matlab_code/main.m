clear all
close all
clc
set(0,'defaultfigurecolor',[1 1 1])
% global g
% g = 9.8;

%% Setting
global state0_parameter AdaptEFonly NaturalAdaptation Euclidean_enhanced Euclidean_projection dt PeriodDesiredTraj LoadShape TrajAmplitude 


% important variables
% NaturalAdaptation = false;                           % Natural vs. Euclidean
% AdaptEFonly = false;                                     %  EF only vs. Entire
% WithLoad = false;                                           % Loaded vs. Unloaded
% Euclidean_enhanced = false;                               % Mahalnobis+Stop at boundary vs. Original
% LoadShape = 0;  % 0: Sphere, 1: parallelepiped, 2:asymmetric arbitrary 
% TrajAmplitude = 0.8;
SectionA = true; 
SectionB = true;
ShowVideo = false;

% % not important variables
% n_time = 2000;
% end_time = 25;
% dt = end_time/(n_time-1);
% PeriodDesiredTraj = 5;

rad2deg = 180/pi;

%% Robot initialization

% if(WithLoad)
%     robot = wam7robot_with_load;
% else
%     robot = wam7robot;
% end


% color = rand(robot.nDOF,3);
color = [   0.4226    0.0244    0.2405
            0.3596    0.2902    0.7639
            0.5583    0.3175    0.7593
            0.7425    0.6537    0.7406
            0.4243    0.9569    0.7437
            0.4294    0.9357    0.1059
            0.1249    0.4579    0.6816];

% Initial parameter
% tspan = linspace(0,end_time,n_time);
% state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
% [state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory_sample(0, 0.8, robot_0);  %% setting inital joint angles and velocities with error zero
% state0_parameter = zeros(robot_0.nDOF*10,1); % set of inertial parameters
% for i =1 : robot.nDOF
%     state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot_0.link(i).J);
% end
% state0_augmented = [state0_joint;state0_parameter];

% Make desired trajectory
% q_desired = zeros(robot.nDOF,n_time);
% q_desired_dot = zeros(robot.nDOF,n_time);
% q_desired_ddot = zeros(robot.nDOF,n_time);
% for j = 1 : n_time
%     [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), 0.8, robot);
% end

% plot in zero position
% theta = zeros(robot.nDOF,1);theta(1) = pi/2;
% [T, Tsave] = forkine(robot,theta);
% G = zeros(6,6,robot.nDOF);
% for j =1 :robot.nDOF
%     G(:,:,j) = robot.link(j).J;
% end
% plot_inertiatensor(Tsave, G, 0.1, color); hold on;
% draw_SE3(eye(4,4));
% for i = 1 : robot.nDOF
%     draw_SE3(Tsave(:,:,i));
% end
% hold off;

%% simluation
if(SectionA)
    disp('----------------------Section A----------------------')
    % initialize
    AdaptEFonly = false;                                     %  EF only vs. Entire
    WithLoad = false;                                           % Loaded vs. Unloaded
    TrajAmplitude = 0.8;
    end_time = 25;
    n_time = end_time * 10000;
    PeriodDesiredTraj = 5;
    robot = wam7robot;
    robot_0 = wam7robot_0;
    
    nTraj = end_time/PeriodDesiredTraj;
    TrajStepsize = n_time / nTraj;
    dt = end_time/(n_time-1);
    tspan = linspace(0,end_time,n_time);
    state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
    [state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory_sample(0, TrajAmplitude, robot_0);  %% setting inital joint angles and velocities with error zero
    state0_parameter = zeros(robot_0.nDOF*10,1); % set of inertial parameters
    for i =1 : robot.nDOF
        state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot_0.link(i).J);
    end
    state0_augmented = [state0_joint;state0_parameter];
    
    % Make desired trajectory
    q_desired = zeros(robot.nDOF,n_time);
    q_desired_dot = zeros(robot.nDOF,n_time);
    q_desired_ddot = zeros(robot.nDOF,n_time);
    for j = 1 : n_time
        [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), 0.8, robot);
    end
    
    % Method 1: Euclidean origin
    Euclidean_enhanced = false;                    % Stop at boundary vs. Original
    NaturalAdaptation = false;                           % Natural vs. Euclidean
    Euclidean_projection = false;
    % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    TrackingError_1 = state_augmented(:,1:robot.nDOF)-q_desired';
    disp('Euc-orign')
    
    % Method 2: Euclidean projection
    Euclidean_enhanced = false;                    % Stop at boundary vs. Original
    NaturalAdaptation = false;                           % Natural vs. Euclidean
    Euclidean_projection = true;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    TrackingError_2 = state_augmented(:,1:robot.nDOF)-q_desired';
    disp('Euc-project')
    
    % Method 3: Euclidean enhanced
    Euclidean_enhanced = true;                    % Stop at boundary vs. Original
    NaturalAdaptation = false;                           % Natural vs. Euclidean
    Euclidean_projection = true;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    TrackingError_3 = state_augmented(:,1:robot.nDOF)-q_desired';
    disp('Euc-enhanced')
    
    % Method 4: Natural
    Euclidean_enhanced = false;                    % Stop at boundary vs. Original
    NaturalAdaptation = true;                           % Natural vs. Euclidean
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    TrackingError_4 = state_augmented(:,1:robot.nDOF)-q_desired';
    disp('Natural')
    
    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    %% Section A. Entire adaptation
    %% plot in zero position
    
    theta = zeros(robot.nDOF,1);theta(1) = pi/2;
    [T, Tsave] = forkine(robot,theta);
    G = zeros(6,6,robot.nDOF);
    for j =1 :robot.nDOF
        G(:,:,j) = robot.link(j).J;
    end
    figure(100)
    plot_inertiatensor(Tsave, G, 0.5, color); hold on;
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    title('Robot inertia in zero-position')
    
    %% Tracking error plot
    % % Tracking of each joint
    % for i = 1 : robot.nDOF
    %     figure(i);
    %     plot(t, state_augmented(:,i));
    %     hold on;
    %     plot(t, q_desired(i,:));
    % end
    %
    % % Tracking Error plot: transient
    % figure(101); hold on;
    % title('Tracking error (joint angle)')
    % ylabel('Error (rad)')
    % xlabel('Time (sec)')
    % TrackingError = zeros(size(q_desired'));
    % for i = 1 : robot.nDOF
    %     TrackingError(:,i) = state_augmented(:,i)-q_desired(i,:)';
    %     plot(t, TrackingError);
    % end
    % legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7')
    %
    % % Tracking error norm: transient
    % errorNorm = sqrt(sum(TrackingError.^2,2)) / sqrt(robot.nDOF);
    % figure(102)
    % plot(t, errorNorm);
    % title('Tracking error (Total)')
    % ylabel('Error (deg)')
    % xlabel('Time (sec)')
    
    %% Total error per rounds (Cyclic)
    RMSErrorPerRounds = zeros(4, nTraj);
    for i=1:nTraj
        errorNorm = sqrt(sum(TrackingError_1.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds(1,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_2.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds(2,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_3.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds(3,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_4.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds(4,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
    end
    
    figure(103)
    bar((1:nTraj)',RMSErrorPerRounds')
    title('Tracking error by round (Repeated sequence)')
    xlabel('Rounds')
    ylabel('RMS error (deg)')
    ylim([0 1.5])
    legend('Euc-origin', 'Euc-project', 'Euc-enhanced', 'Natural')
    
    %% Generalizability: RMS Error by trajectory rounds (Sequence)
    
     % initialize
     AdaptEFonly = false;                                     %  EF only vs. Entire
     WithLoad = false;                                           % Loaded vs. Unloaded
     robot = wam7robot;
     robot_0 = wam7robot_0;
     
     PeriodDesiredTraj = 5;
     n_time_sample = PeriodDesiredTraj * 10000;
     dt = PeriodDesiredTraj/(n_time_sample-1);
     tspan_sample = linspace(0,PeriodDesiredTraj,n_time_sample);
        
     % make a sequence of 5 sample trajectories
     q_desired_1 = zeros(robot.nDOF,n_time_sample);
     q_desired_dot_1 = zeros(robot.nDOF,n_time_sample);
     q_desired_ddot_1 = zeros(robot.nDOF,n_time_sample);
     q_desired_2 = zeros(robot.nDOF,n_time_sample);
     q_desired_dot_2 = zeros(robot.nDOF,n_time_sample);
     q_desired_ddot_2 = zeros(robot.nDOF,n_time_sample);
     q_desired_3 = zeros(robot.nDOF,n_time_sample);
     q_desired_dot_3 = zeros(robot.nDOF,n_time_sample);
     q_desired_ddot_3 = zeros(robot.nDOF,n_time_sample);
     q_desired_4 = zeros(robot.nDOF,n_time_sample);
     q_desired_dot_4 = zeros(robot.nDOF,n_time_sample);
     q_desired_ddot_4 = zeros(robot.nDOF,n_time_sample);
     q_desired_5 = zeros(robot.nDOF,n_time_sample);
     q_desired_dot_5 = zeros(robot.nDOF,n_time_sample);
     q_desired_ddot_5 = zeros(robot.nDOF,n_time_sample);
     A1 = 0.4;
     A2 = 0.6;
     A3 = 0.8;
     A4 = 1.0;
     A5 = 1.2;
     for j = 1 : n_time_sample
         [q_desired_1(:,j), q_desired_dot_1(:,j), q_desired_ddot_1(:,j)] = make_desired_trajectory_sample(tspan_sample(j), A1, robot);
         [q_desired_2(:,j), q_desired_dot_2(:,j), q_desired_ddot_2(:,j)] = make_desired_trajectory_sample(tspan_sample(j), A2, robot);
         [q_desired_3(:,j), q_desired_dot_3(:,j), q_desired_ddot_3(:,j)] = make_desired_trajectory_sample(tspan_sample(j), A3, robot);
         [q_desired_4(:,j), q_desired_dot_4(:,j), q_desired_ddot_4(:,j)] = make_desired_trajectory_sample(tspan_sample(j), A4, robot);
         [q_desired_5(:,j), q_desired_dot_5(:,j), q_desired_ddot_5(:,j)] = make_desired_trajectory_sample(tspan_sample(j), A5, robot);
     end
     
     state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
     [state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory_sample(0, A1, robot_0);  %% setting inital joint angles and velocities with error zero
     state0_parameter = zeros(robot_0.nDOF*10,1); % set of inertial parameters
     for i =1 : robot.nDOF
         state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot_0.link(i).J);
     end
     state0_augmented = [state0_joint;state0_parameter];
    
    % Run
    RMSErrorPerRounds_sample = zeros(4, 5);
    Deadzone = zeros(4, 5);
    Euclidean_projection_cell   = {false, true, true, false};
    Euclidean_enhanced_cell     = {false, false, true, false};% Mahalnobis+Stop at boundary vs. Original
    NaturalAdaptation_cell      = {false, false, false, true};% Natural vs. Euclidean
    for expNumb=1:4
        Euclidean_enhanced      = Euclidean_enhanced_cell{expNumb};
        NaturalAdaptation       = NaturalAdaptation_cell{expNumb};
        Euclidean_projection    = Euclidean_projection_cell{expNumb};
        disp(['----------------------' num2str(expNumb) '----------------------'])
        
        state0_augmented_1 = state0_augmented;
%         [t_1,state_augmented_1] = ode45(@(t,state_augmented) Dynamics_Adaptive_sample(t, state_augmented, robot,A1), tspan_sample, state0_augmented_1);
%         [t_2,state_augmented_2] = ode45(@(t,state_augmented) Dynamics_Adaptive_sample(t, state_augmented, robot,A2), tspan_sample, state_augmented_1(end,:)');
%         [t_3,state_augmented_3] = ode45(@(t,state_augmented) Dynamics_Adaptive_sample(t, state_augmented, robot,A3), tspan_sample, state_augmented_2(end,:)');
%         [t_4,state_augmented_4] = ode45(@(t,state_augmented) Dynamics_Adaptive_sample(t, state_augmented, robot,A4), tspan_sample, state_augmented_3(end,:)');
%         [t_5,state_augmented_5] = ode45(@(t,state_augmented) Dynamics_Adaptive_sample(t, state_augmented, robot,A5), tspan_sample, state_augmented_4(end,:)');
        TrajAmplitude = A1;
        [t_1,state_augmented_1] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state0_augmented_1);
        disp(['Traj 1'])
        TrajAmplitude = A2;
        [t_2,state_augmented_2] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_1(end,:)');
        disp(['Traj 2'])
        TrajAmplitude = A3;
        [t_3,state_augmented_3] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_2(end,:)');
        disp(['Traj 3'])
        TrajAmplitude = A4;
        [t_4,state_augmented_4] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_3(end,:)');
        disp(['Traj 4'])
        TrajAmplitude = A5;
        [t_5,state_augmented_5] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_4(end,:)');
        disp(['Traj 5'])

        TrackingError_sample1 = state_augmented_1(:,1:robot.nDOF)-q_desired_1';
        TrackingError_sample2 = state_augmented_2(:,1:robot.nDOF)-q_desired_2';
        TrackingError_sample3 = state_augmented_3(:,1:robot.nDOF)-q_desired_3';
        TrackingError_sample4 = state_augmented_4(:,1:robot.nDOF)-q_desired_4';
        TrackingError_sample5 = state_augmented_5(:,1:robot.nDOF)-q_desired_5';
        Deadzone_sample1 = ComputeDeadZone(state_augmented_1(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        Deadzone_sample2 = ComputeDeadZone(state_augmented_2(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        Deadzone_sample3 = ComputeDeadZone(state_augmented_3(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        Deadzone_sample4 = ComputeDeadZone(state_augmented_4(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        Deadzone_sample5 = ComputeDeadZone(state_augmented_5(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        
        % Total error per rounds
        errorNorm = sqrt(sum(TrackingError_sample1.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_sample(expNumb,1) = rad2deg * rms(errorNorm);
        errorNorm = sqrt(sum(TrackingError_sample2.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_sample(expNumb,2) = rad2deg * rms(errorNorm);
        errorNorm = sqrt(sum(TrackingError_sample3.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_sample(expNumb,3) = rad2deg * rms(errorNorm);
        errorNorm = sqrt(sum(TrackingError_sample4.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_sample(expNumb,4) = rad2deg * rms(errorNorm);
        errorNorm = sqrt(sum(TrackingError_sample5.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_sample(expNumb,5) = rad2deg * rms(errorNorm);
        
        % Deadzone time ratio
        Deadzone(expNumb,1) = size(Deadzone_sample1,1) / n_time_sample;
        Deadzone(expNumb,2) = size(Deadzone_sample2,1) / n_time_sample;
        Deadzone(expNumb,3) = size(Deadzone_sample3,1) / n_time_sample;
        Deadzone(expNumb,4) = size(Deadzone_sample4,1) / n_time_sample;
        Deadzone(expNumb,5) = size(Deadzone_sample5,1) / n_time_sample;
    end
    
    % Tracking error plot
    figure(104)
    bar((1:5)',RMSErrorPerRounds_sample')
    title('Tracking error by round (Varied sequence)')
    xlabel('Trajectory sequence')
    ylabel('RMS error (deg)')
    ylim([0 1.5])
    legend('Euc-origin', 'Euc-project', 'Euc-enhanced', 'Natural')
    
    % Deadzone plot
    figure(105)
    bar((1:5)',Deadzone')
    title('Deadzone on a sequence of trajectories')
    xlabel('Trajectory sequence')
    ylabel('Time duration')
    legend('Euc-origin', 'Euc-project', 'Euc-enhanced', 'Natural')
end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% Section B. Adapt load on EF
if(SectionB)
    disp('------------------Section B------------------')
    % important variables
    AdaptEFonly = true;                                     %  EF only vs. Entire
    WithLoad = true;                                           % Loaded vs. Unloaded
    LoadShape_cell = {0,1,2};  % 0: Sphere, 1: parallelepiped, 2:asymmetric arbitrary
    LoadShape_string = {'Sphere','Parallelepiped','Arbitrary'};
    end_time = 25;
    n_time = end_time * 10000;
    dt = end_time/(n_time-1);
    PeriodDesiredTraj = 5;
    TrajAmplitude = 0.8;
    nTraj = end_time/PeriodDesiredTraj;
    TrajStepsize = n_time / nTraj;
    tspan = linspace(0,end_time,n_time);
    
    % Make desired trajectory
    q_desired = zeros(robot.nDOF,n_time);
    q_desired_dot = zeros(robot.nDOF,n_time);
    q_desired_ddot = zeros(robot.nDOF,n_time);
    for j = 1 : n_time
        [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), TrajAmplitude, robot);
    end
    
    % Run
    RMSErrorPerRounds = cell(1,3);
    RMSErrorPerRounds{1} = zeros(4, nTraj);
    RMSErrorPerRounds{2} = zeros(4, nTraj);
    RMSErrorPerRounds{3} = zeros(4, nTraj);
    state_augmented_1 = cell(1,3);
    state_augmented_2 = cell(1,3);
    state_augmented_3 = cell(1,3);
    state_augmented_4 = cell(1,3);
    
    for k=1:3 % size(LoadShape_cell,2)
        LoadShape = LoadShape_cell{k};
        disp([num2str(k) 'th LoadShape = ' num2str(LoadShape)])
        % Initial parameter
        robot0 = wam7robot_0;
        robot = wam7robot_with_load;
        state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
        [state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory_sample(0, TrajAmplitude, robot0);  %% setting inital joint angles and velocities with error zero
        state0_parameter = zeros(robot0.nDOF*10,1); % set of inertial parameters
        for i =1 : robot0.nDOF
            state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot0.link(i).J);
        end
        state0_augmented = [state0_joint;state0_parameter];
        
        % Method 1: Euclidean origin
        Euclidean_enhanced = false;                    % Stop at boundary vs. Original
        NaturalAdaptation = false;                           % Natural vs. Euclidean
        Euclidean_projection = false;
        %     [t,state_augmented_1] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [t, state_augmented_1{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        TrackingError_1 = state_augmented_1{k}(:,1:robot.nDOF)-q_desired';
        disp('Euc-orign')
        
        % Method 2: Euclidean projection
        Euclidean_enhanced = false;                    % Stop at boundary vs. Original
        NaturalAdaptation = false;                           % Natural vs. Euclidean
        Euclidean_projection = true;
        %     [~,state_augmented_2] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_2{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        TrackingError_2 = state_augmented_2{k}(:,1:robot.nDOF)-q_desired';
        disp('Euc-project')
        
        % Method 3: Euclidean enhanced
        Euclidean_enhanced = true;                    % Stop at boundary vs. Original
        NaturalAdaptation = false;                           % Natural vs. Euclidean
        Euclidean_projection = true;
        %     [~,state_augmented_2] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_3{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        TrackingError_3 = state_augmented_3{k}(:,1:robot.nDOF)-q_desired';
        disp('Euc-enhanced')
        
        % Method 4: Natural
        Euclidean_enhanced = false;                    % Stop at boundary vs. Original
        NaturalAdaptation = true;                           % Natural vs. Euclidean
        Euclidean_projection = false;
        %     [~,state_augmented_3] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_4{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        TrackingError_4 = state_augmented_4{k}(:,1:robot.nDOF)-q_desired';
        disp('Natural')
        
        % Total error per rounds (Cyclic)
        for i=1:nTraj
            errorNorm_1{k} = sqrt(sum(TrackingError_1.^2,2)) / sqrt(robot.nDOF);
            RMSErrorPerRounds{k}(1,i) = rad2deg * rms(errorNorm_1{k}(TrajStepsize*(i-1)+1:TrajStepsize*i));
            errorNorm_2{k} = sqrt(sum(TrackingError_2.^2,2)) / sqrt(robot.nDOF);
            RMSErrorPerRounds{k}(2,i) = rad2deg * rms(errorNorm_2{k}(TrajStepsize*(i-1)+1:TrajStepsize*i));
            errorNorm_3{k} = sqrt(sum(TrackingError_3.^2,2)) / sqrt(robot.nDOF);
            RMSErrorPerRounds{k}(3,i) = rad2deg * rms(errorNorm_3{k}(TrajStepsize*(i-1)+1:TrajStepsize*i));
            errorNorm_4{k} = sqrt(sum(TrackingError_4.^2,2)) / sqrt(robot.nDOF);
            RMSErrorPerRounds{k}(4,i) = rad2deg * rms(errorNorm_4{k}(TrajStepsize*(i-1)+1:TrajStepsize*i));
        end
    end
    
    % plots
    for k=1:3
        figure(106 + k)
        bar((1:nTraj)',RMSErrorPerRounds{k}(:,:)')
        title(['Tracking error by round: ' LoadShape_string{k}])
        xlabel('Rounds')
        ylabel('RMS error (deg)')
        ylim([0 5])
        legend('Euc-origin', 'Euc-project', 'Euc-enhanced', 'Natural')
        
        figure(120+k)
        plot(tspan, [errorNorm_1{k}';errorNorm_2{k}';errorNorm_3{k}';errorNorm_4{k}']);
        title(['Transient tacking error: ' LoadShape_string{k}])
        xlabel('Time (sec)')
        ylabel('RMS error (deg)')
        ymax = 0.15;
        ylim([0 ymax])
        legend('Euc-origin', 'Euc-project', 'Euc-enhanced', 'Natural')
        hold on;
        for i=1:4
            y_dotted = 0:0.01:ymax; 
            x_dotted = i * PeriodDesiredTraj * ones(size(y_dotted));
            plot(x_dotted, y_dotted,'k--')
        end
        hold off;
    end

end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% Adapting video
if(ShowVideo)
    G = zeros(6,6,robot.nDOF);
    for j =1 :robot.nDOF
        G(:,:,j) = robot.link(j).J;
    end
    for k =1 :50:  size(state_augmented,1)
        
        theta = state_augmented(k,1:robot.nDOF);
        [T, Tsave] = forkine(robot,theta);
        G_out = p2G(state_augmented(k,2*robot.nDOF+1:end)');
        aa = figure(10);
        set(aa, 'position',[150 150 1800 500]);
                % plot in zero position
        c = subplot(1,3,1);
        G = zeros(6,6,robot.nDOF);
        for j =1 :robot.nDOF
            G(:,:,j) = robot.link(j).J;
        end
        plot_inertiatensor(Tsave, G, 0.7, color); hold on;
        draw_SE3(eye(4,4));
        for i = 1 : robot.nDOF
            draw_SE3(Tsave(:,:,i));
        end
        axis([-1.0 1.0 -1.0 1.0 -1.0 1.0]);
        set(c,'position',[0.04 0.1 0.25,0.8]);
%         hold off;
        
        a = subplot(1,3,2);
        plot_inertiatensor(Tsave, G_out, 0.7, color);hold on;
        draw_SE3(eye(4,4));
        for i = 1 : robot.nDOF
            draw_SE3(Tsave(:,:,i));
        end
        axis([-1.0 1.0 -1.0 1.0 -1.0 1.0]);
        set(a,'position',[0.37 0.1 0.25,0.8]);
        % hold off;
        b = subplot(1,3,3);
        hold on; ylim([-2 2]);
        y = [reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1)];
        bar(y);
        set(b,'position',[0.7 0.1 0.25,0.8]);
        set(b, 'xtick', [1:robot.nDOF], 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
        drawnow;
        

    end
    G_out = p2G(state_augmented(end,2*robot.nDOF+1:end)');
    plot_inertiatensor(Tsave(:,:,end), G_out(:,:,end), 0.7, color(end,:));hold on;
    plot_inertiatensor(Tsave(:,:,end), G(:,:,end), 0.7, color(end,:));hold on;
end