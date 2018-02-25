clear all
close all
% clc
set(0,'defaultfigurecolor',[1 1 1],'defaultLineLineWidth',2)
% global g
% g = 9.8;

%% Setting
global state0_parameter AdaptEFonly NaturalAdaptation Euclidean_enhanced Euclidean_projection dt 
global PeriodDesiredTraj LoadShape TrajAmplitude bBadInitial NoAdaptation GaussianInitial

% important variables
SectionA = true; 
% SectionB = true;
% ShowVideo = false;
% bBadInitial = true;
GaussianInitial = true;

% % not important variables
rad2deg = 180/pi;

% color = rand(robot.nDOF,3);
color = [   0.4226    0.0244    0.2405
            0.3596    0.2902    0.7639
            0.5583    0.3175    0.7593
            0.7425    0.6537    0.7406
            0.4243    0.9569    0.7437
            0.4294    0.9357    0.1059
            0.1249    0.4579    0.6816];


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Gaussian initialization for Monte-Carlo test
if(GaussianInitial)
    disp('----------------Gaussian initial---------------')
    % initialize
    AdaptEFonly = false;                                     %  EF only vs. Entire
    WithLoad = false;                                           % Loaded vs. Unloaded
    TrajAmplitude = 0.8;
    end_time = 5;
    n_time = end_time * 10000;
    PeriodDesiredTraj = end_time;
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
    
    % Method 1: No adaptation
    NoAdaptation            = true;
    Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
    NaturalAdaptation       = false;                           % Natural vs. Euclidean
    Euclidean_projection    = false;
    % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_no] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_no))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_1 = state_augmented_no(:,1:robot.nDOF)-q_desired';
    disp('No-adaptation')
    
    % Method 2: Euclidean
    NoAdaptation                = false;
    Euclidean_enhanced          = false;                    % Stop at boundary vs. Original
    NaturalAdaptation           = false;                           % Natural vs. Euclidean
    Euclidean_projection        = false;
    % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_euc] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_euc))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_2 = state_augmented_euc(:,1:robot.nDOF)-q_desired';
    disp('Euclidean')
    
    % Method 3: Const. pullback
    NoAdaptation            = false;
    Euclidean_enhanced      = true;                    % Stop at boundary vs. Original
    NaturalAdaptation       = false;                           % Natural vs. Euclidean
    Euclidean_projection    = false;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_pb] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_pb))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_3 = state_augmented_pb(:,1:robot.nDOF)-q_desired';
    disp('Const. pullback')
    
    % Method 4: Natural
    NoAdaptation                = false;
    Euclidean_enhanced          = false;                    % Stop at boundary vs. Original
    NaturalAdaptation           = true;                           % Natural vs. Euclidean
    Euclidean_projection        = false;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_nat] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_nat))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_4 = state_augmented_nat(:,1:robot.nDOF)-q_desired';
    disp('Natural')
    
    %% Save the result
    RMSerror = zeros(1,4);
    RMSerror(1) = rad2deg * rms(rms(TrackingError_1));
    RMSerror(2) = rad2deg * rms(rms(TrackingError_2));
    RMSerror(3) = rad2deg * rms(rms(TrackingError_3));
    RMSerror(4) = rad2deg * rms(rms(TrackingError_4));
    
    fid = fopen(['SectionA_GaussInit_' date '.txt'], 'a');
    fprintf(fid, '%5.3f %5.3f %5.3f %5.3f\n', RMSerror');
    return;
end

%% Section A. Entire adaptation
if(SectionA)
    disp('----------------------Section A----------------------')
    % initialize
    GaussianInitial = false;
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
    
    % Method 1: No adaptation
    NoAdaptation            = true;
    Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
    NaturalAdaptation       = false;                           % Natural vs. Euclidean
    Euclidean_projection    = false;
    % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_no] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_no))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_1 = state_augmented_no(:,1:robot.nDOF)-q_desired';
    disp('No-adaptation')
    
    % Method 2: Euclidean
    NoAdaptation            = false;
    Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
    NaturalAdaptation       = false;                           % Natural vs. Euclidean
    Euclidean_projection 	= false;
    % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_euc] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_euc))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_2 = state_augmented_euc(:,1:robot.nDOF)-q_desired';
    disp('Euclidean')
    
    % Method 3: Const. pullback
    NoAdaptation            = false;
    Euclidean_enhanced      = true;                    % Stop at boundary vs. Original
    NaturalAdaptation       = false;                           % Natural vs. Euclidean
    Euclidean_projection    = false;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_pb] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_pb))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_3 = state_augmented_pb(:,1:robot.nDOF)-q_desired';
    disp('Const. pullback')
    
    % Method 4: Natural
    NoAdaptation            = false;
    Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
    NaturalAdaptation       = true;                           % Natural vs. Euclidean
    Euclidean_projection    = false;
    % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
    [~, state_augmented_nat] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
    if(isnan(state_augmented_nat))
        disp('------------------Main returned------------------')
        return;
    end
    TrackingError_4 = state_augmented_nat(:,1:robot.nDOF)-q_desired';
    disp('Natural')
    
    
    %% plot in zero position
%     theta = zeros(robot.nDOF,1);theta(1) = pi/2;
%     [T, Tsave] = forkine(robot,theta);
%     G = zeros(6,6,robot.nDOF);
%     for j =1 :robot.nDOF
%         G(:,:,j) = robot.link(j).J;
%     end
%     figure(100)
%     plot_inertiatensor(Tsave, G, 0.5, color); hold on;
%     draw_SE3(eye(4,4));
%     for i = 1 : robot.nDOF
%         draw_SE3(Tsave(:,:,i));
%     end
%     title('Robot inertia in zero-position')
    
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
    RMSErrorPerRounds_repeated = zeros(4, nTraj);
    for i=1:nTraj
        errorNorm = sqrt(sum(TrackingError_1.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_repeated(1,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_2.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_repeated(2,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_3.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_repeated(3,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
        errorNorm = sqrt(sum(TrackingError_4.^2,2)) / sqrt(robot.nDOF);
        RMSErrorPerRounds_repeated(4,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
    end
    
    figure(103)
    bar((1:nTraj)',RMSErrorPerRounds_repeated')
    title('Tracking error by round (Repeated sequence)')
    xlabel('Rounds')
    ylabel('RMS error (deg)')
    ylim([0 5])
    legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
    drawnow
    
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
    NoAdaptation_cell           = {true,  false, false, false};
    Euclidean_projection_cell   = {false, false, false, false};
    Euclidean_enhanced_cell     = {false, false, true, false};% Mahalnobis+Stop at boundary vs. Original
    NaturalAdaptation_cell      = {false, false, false, true};% Natural vs. Euclidean
    for expNumb=1:4
        NoAdaptation            = NoAdaptation_cell{expNumb};
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
    ylim([0 5])
    legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
    drawnow
    
    % Deadzone plot
    figure(105)
    bar((1:5)',Deadzone')
    title('Physically-inconsistent duration')
    xlabel('Trajectory sequence')
    ylabel('Time duration')
    legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
    drawnow
end

%% Section B. Adapt load on EF
if(SectionB)
    disp('------------------Section B------------------')
    % important variables
    GaussianInitial = false;
    AdaptEFonly = true;                                     %  EF only vs. Entire
    WithLoad = true;                                           % Loaded vs. Unloaded
    LoadShape_cell = {0,1,2};  % 0: Sphere, 1: parallelepiped, 2:ellispoid with offset
    LoadShape_string = {'Sphere','Parallelepiped','Ellipsoid with offset'};
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
    state_augmented_no = cell(1,3);
    state_augmented_euc = cell(1,3);
    state_augmented_pb = cell(1,3);
    state_augmented_nat = cell(1,3);
    
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
        
        % Method 1: No adaptation
        NoAdaptation            = true;
        Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
        NaturalAdaptation       = false;                           % Natural vs. Euclidean
        Euclidean_projection    = false;
        %     [t,state_augmented_1] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_no{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_no{k}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_1 = state_augmented_no{k}(:,1:robot.nDOF)-q_desired';
        disp('No-adaptation')
        
        % Method 2: Euclidean
        NoAdaptation            = false;
        Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
        NaturalAdaptation       = false;                           % Natural vs. Euclidean
        Euclidean_projection    = false;
        %     [~,state_augmented_2] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_euc{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_euc{k}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_2 = state_augmented_euc{k}(:,1:robot.nDOF)-q_desired';
        disp('Euclidean')
        
        % Method 3: Const. pullback
        NoAdaptation            = false;
        Euclidean_enhanced      = true;                    % Stop at boundary vs. Original
        NaturalAdaptation       = false;                           % Natural vs. Euclidean
        Euclidean_projection    = false;
        %     [~,state_augmented_2] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_pb{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_pb{k}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_3 = state_augmented_pb{k}(:,1:robot.nDOF)-q_desired';
        disp('Const. pullback')
        
        % Method 4: Natural
        NoAdaptation            = false;
        Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
        NaturalAdaptation       = true;                           % Natural vs. Euclidean
        Euclidean_projection    = false;
        %     [~,state_augmented_3] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_nat{k}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_nat{k}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_4 = state_augmented_nat{k}(:,1:robot.nDOF)-q_desired';
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
        legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
        
        figure(120+k)
        plot(tspan, [errorNorm_1{k}';errorNorm_2{k}';errorNorm_3{k}';errorNorm_4{k}']);
        title(['Transient tacking error: ' LoadShape_string{k}])
        xlabel('Time (sec)')
        ylabel('RMS error (deg)')
        ymax = 1;
        ylim([0 ymax])
        legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
        hold on;
        for i=1:4
            y_dotted = 0:0.01:ymax;
            x_dotted = i * PeriodDesiredTraj * ones(size(y_dotted));
            plot(x_dotted, y_dotted,'k--','LineWidth',1)
        end
        hold off;
    end
    
end

