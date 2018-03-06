clear all
close all
% clc
set(0,'defaultfigurecolor',[1 1 1],'defaultLineLineWidth',1)
rng('shuffle');
% global g
% g = 9.8;

%% Setting
global state0_parameter AdaptEFonly NaturalAdaptation Euclidean_enhanced Euclidean_projection dt 
global PeriodDesiredTraj LoadShape TrajAmplitude bBadInitial NoAdaptation GaussianInitial

% important variables
SectionRandom = true;
SectionA = false; 
SectionB = false;

ShowVideo = false;
bBadInitial = false;
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
if(SectionRandom)
    maxIter = 1 ;
    state_augmented_no_entire = cell(1,maxIter);
    state_augmented_euc_entire = cell(1,maxIter);
    state_augmented_pb_entire = cell(1,maxIter);
    state_augmented_nat_entire = cell(1,maxIter);
    for iter = 1:maxIter
        disp(['----------------' num2str(iter) ' iteration ---------------'])
        % initialize
        AdaptEFonly = false;                                     %  EF only vs. Entire
        WithLoad = false;                                           % Loaded vs. Unloaded
        TrajAmplitude = 0.8;
        end_time = 10;
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
        [~, state_augmented_no_entire{iter}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_no_entire{iter}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_1 = state_augmented_no_entire{iter}(:,1:robot.nDOF)-q_desired';
        disp('No-adaptation')
        
        % Method 2: Euclidean
        NoAdaptation                = false;
        Euclidean_enhanced          = false;                    % Stop at boundary vs. Original
        NaturalAdaptation           = false;                           % Natural vs. Euclidean
        Euclidean_projection        = false;
        % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_euc_entire{iter}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_euc_entire{iter}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_2 = state_augmented_euc_entire{iter}(:,1:robot.nDOF)-q_desired';
        disp('Euclidean')
        
        % Method 3: Const. pullback
        NoAdaptation            = false;
        Euclidean_enhanced      = true;                    % Stop at boundary vs. Original
        NaturalAdaptation       = false;                           % Natural vs. Euclidean
        Euclidean_projection    = false;
        % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_pb_entire{iter}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_pb_entire{iter}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_3 = state_augmented_pb_entire{iter}(:,1:robot.nDOF)-q_desired';
        disp('Const. pullback')
        
        % Method 4: Natural
        NoAdaptation                = false;
        Euclidean_enhanced          = false;                    % Stop at boundary vs. Original
        NaturalAdaptation           = true;                           % Natural vs. Euclidean
        Euclidean_projection        = false;
        % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
        [~, state_augmented_nat_entire{iter}] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
        if(isnan(state_augmented_nat_entire{iter}))
            disp('------------------Main returned------------------')
            return;
        end
        TrackingError_4 = state_augmented_nat_entire{iter}(:,1:robot.nDOF)-q_desired';
        disp('Natural')
        
        %% Save the result
%         RMSerror = zeros(1,4);
%         RMSerror(1) = rad2deg * rms(rms(TrackingError_1));
%         RMSerror(2) = rad2deg * rms(rms(TrackingError_2));
%         RMSerror(3) = rad2deg * rms(rms(TrackingError_3));
%         RMSerror(4) = rad2deg * rms(rms(TrackingError_4));
%         
%         fid = fopen(['SectionA_GaussInit_Noise35_' date '.txt'], 'a');
%         fprintf(fid, '%5.3f %5.3f %5.3f %5.3f\n', RMSerror');
    end    
    save('FixedNoise04v2');
    return;
end

%% Section A. Entire adaptation
if(SectionA)
    disp('----------------------Section A----------------------')
%     % initialize
%     GaussianInitial = false;
%     AdaptEFonly = false;                                     %  EF only vs. Entire
%     WithLoad = false;                                           % Loaded vs. Unloaded
%     TrajAmplitude = 0.8;
%     end_time = 25;
%     n_time = end_time * 10000;
%     PeriodDesiredTraj = 5;
%     robot = wam7robot;
%     robot_0 = wam7robot_0;
%     
%     nTraj = end_time/PeriodDesiredTraj;
%     TrajStepsize = n_time / nTraj;
%     dt = end_time/(n_time-1);
%     tspan = linspace(0,end_time,n_time);
%     state0_joint = zeros(robot.nDOF*2,1); % [joint_angle; joint_velocity]
%     [state0_joint(1:robot.nDOF,1), state0_joint(robot.nDOF+1:robot.nDOF*2,1)] = make_desired_trajectory_sample(0, TrajAmplitude, robot_0);  %% setting inital joint angles and velocities with error zero
%     state0_parameter = zeros(robot_0.nDOF*10,1); % set of inertial parameters
%     for i =1 : robot.nDOF
%         state0_parameter(10*(i-1)+1:10*i,1) = G2p(robot_0.link(i).J);
%     end
%     state0_augmented = [state0_joint;state0_parameter];
%     
%     % Make desired trajectory
%     q_desired = zeros(robot.nDOF,n_time);
%     q_desired_dot = zeros(robot.nDOF,n_time);
%     q_desired_ddot = zeros(robot.nDOF,n_time);
%     for j = 1 : n_time
%         [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), 0.8, robot);
%     end
%     
%     % Method 1: No adaptation
%     NoAdaptation            = true;
%     Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
%     NaturalAdaptation       = false;                           % Natural vs. Euclidean
%     Euclidean_projection    = false;
%     % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
%     [~, state_augmented_no] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
%     if(isnan(state_augmented_no))
%         disp('------------------Main returned------------------')
%         return;
%     end
%     TrackingError_1 = state_augmented_no(:,1:robot.nDOF)-q_desired';
%     disp('No-adaptation')
%     
%     % Method 2: Euclidean
%     NoAdaptation            = false;
%     Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
%     NaturalAdaptation       = false;                           % Natural vs. Euclidean
%     Euclidean_projection 	= false;
%     % [t,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
%     [~, state_augmented_euc] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
%     if(isnan(state_augmented_euc))
%         disp('------------------Main returned------------------')
%         return;
%     end
%     TrackingError_2 = state_augmented_euc(:,1:robot.nDOF)-q_desired';
%     disp('Euclidean')
%     
%     % Method 3: Const. pullback
%     NoAdaptation            = false;
%     Euclidean_enhanced      = true;                    % Stop at boundary vs. Original
%     NaturalAdaptation       = false;                           % Natural vs. Euclidean
%     Euclidean_projection    = false;
%     % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
%     [~, state_augmented_pb] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
%     if(isnan(state_augmented_pb))
%         disp('------------------Main returned------------------')
%         return;
%     end
%     TrackingError_3 = state_augmented_pb(:,1:robot.nDOF)-q_desired';
%     disp('Const. pullback')
%     
%     % Method 4: Natural
%     NoAdaptation            = false;
%     Euclidean_enhanced      = false;                    % Stop at boundary vs. Original
%     NaturalAdaptation       = true;                           % Natural vs. Euclidean
%     Euclidean_projection    = false;
%     % [~,state_augmented] = ode45(@(t,state_augmented) Dynamics_Adaptive(t, state_augmented, robot), tspan, state0_augmented);
%     [~, state_augmented_nat] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented);
%     if(isnan(state_augmented_nat))
%         disp('------------------Main returned------------------')
%         return;
%     end
%     TrackingError_4 = state_augmented_nat(:,1:robot.nDOF)-q_desired';
%     disp('Natural')
%     
%     
%     %% plot in zero position
% %     theta = zeros(robot.nDOF,1);theta(1) = pi/2;
% %     [T, Tsave] = forkine(robot,theta);
% %     G = zeros(6,6,robot.nDOF);
% %     for j =1 :robot.nDOF
% %         G(:,:,j) = robot.link(j).J;
% %     end
% %     figure(100)
% %     plot_inertiatensor(Tsave, G, 0.5, color); hold on;
% %     draw_SE3(eye(4,4));
% %     for i = 1 : robot.nDOF
% %         draw_SE3(Tsave(:,:,i));
% %     end
% %     title('Robot inertia in zero-position')
%     
%     %% Tracking error plot
%     % % Tracking of each joint
%     % for i = 1 : robot.nDOF
%     %     figure(i);
%     %     plot(t, state_augmented(:,i));
%     %     hold on;
%     %     plot(t, q_desired(i,:));
%     % end
%     %
%     % % Tracking Error plot: transient
%     % figure(101); hold on;
%     % title('Tracking error (joint angle)')
%     % ylabel('Error (rad)')
%     % xlabel('Time (sec)')
%     % TrackingError = zeros(size(q_desired'));
%     % for i = 1 : robot.nDOF
%     %     TrackingError(:,i) = state_augmented(:,i)-q_desired(i,:)';
%     %     plot(t, TrackingError);
%     % end
%     % legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7')
%     %
%     % % Tracking error norm: transient
%     % errorNorm = sqrt(sum(TrackingError.^2,2)) / sqrt(robot.nDOF);
%     % figure(102)
%     % plot(t, errorNorm);
%     % title('Tracking error (Total)')
%     % ylabel('Error (deg)')
%     % xlabel('Time (sec)')
%     
%     %% Total error per rounds (Cyclic)
%     RMSErrorPerRounds_repeated = zeros(4, nTraj);
%     for i=1:nTraj
%         errorNorm = sqrt(sum(TrackingError_1.^2,2)) / sqrt(robot.nDOF);
%         RMSErrorPerRounds_repeated(1,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
%         errorNorm = sqrt(sum(TrackingError_2.^2,2)) / sqrt(robot.nDOF);
%         RMSErrorPerRounds_repeated(2,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
%         errorNorm = sqrt(sum(TrackingError_3.^2,2)) / sqrt(robot.nDOF);
%         RMSErrorPerRounds_repeated(3,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
%         errorNorm = sqrt(sum(TrackingError_4.^2,2)) / sqrt(robot.nDOF);
%         RMSErrorPerRounds_repeated(4,i) = rad2deg * rms(errorNorm(TrajStepsize*(i-1)+1:TrajStepsize*i));
%     end
%     
%     figure(103)
%     bar((1:nTraj)',RMSErrorPerRounds_repeated')
%     title('Tracking error by round (Repeated sequence)')
%     xlabel('Rounds')
%     ylabel('RMS error (deg)')
%     ylim([0 5])
%     legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
%     drawnow
    
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
        if(isnan(state_augmented_1))
            disp('------------------Main returned------------------')
            return;
        end
        disp(['Traj 1'])
        
        TrajAmplitude = A2;
        [t_2,state_augmented_2] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_1(end,:)');
        if(isnan(state_augmented_2))
            disp('------------------Main returned------------------')
            return;
        end
        disp(['Traj 2'])
        
        TrajAmplitude = A3;
        [t_3,state_augmented_3] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_2(end,:)');
        if(isnan(state_augmented_3))
            disp('------------------Main returned------------------')
            return;
        end
        disp(['Traj 3'])
        
        TrajAmplitude = A4;
        [t_4,state_augmented_4] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_3(end,:)');
        if(isnan(state_augmented_4))
            disp('------------------Main returned------------------')
            return;
        end
        disp(['Traj 4'])
        
        TrajAmplitude = A5;
        [t_5,state_augmented_5] = euler_integration_Dynamics_Adaptive(robot, tspan_sample, state_augmented_4(end,:)');
        if(isnan(state_augmented_5))
            disp('------------------Main returned------------------')
            return;
        end
        disp(['Traj 5'])

        TrackingError_sample1 = state_augmented_1(:,1:robot.nDOF)-q_desired_1';
        TrackingError_sample2 = state_augmented_2(:,1:robot.nDOF)-q_desired_2';
        TrackingError_sample3 = state_augmented_3(:,1:robot.nDOF)-q_desired_3';
        TrackingError_sample4 = state_augmented_4(:,1:robot.nDOF)-q_desired_4';
        TrackingError_sample5 = state_augmented_5(:,1:robot.nDOF)-q_desired_5';
%         Deadzone_sample1 = ComputeDeadZone(state_augmented_1(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
%         Deadzone_sample2 = ComputeDeadZone(state_augmented_2(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
%         Deadzone_sample3 = ComputeDeadZone(state_augmented_3(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
%         Deadzone_sample4 = ComputeDeadZone(state_augmented_4(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
%         Deadzone_sample5 = ComputeDeadZone(state_augmented_5(:,end-robot.nDOF*10+1:end),Euclidean_projection,NaturalAdaptation);
        
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
%         Deadzone(expNumb,1) = size(Deadzone_sample1,1) / n_time_sample;
%         Deadzone(expNumb,2) = size(Deadzone_sample2,1) / n_time_sample;
%         Deadzone(expNumb,3) = size(Deadzone_sample3,1) / n_time_sample;
%         Deadzone(expNumb,4) = size(Deadzone_sample4,1) / n_time_sample;
%         Deadzone(expNumb,5) = size(Deadzone_sample5,1) / n_time_sample;
    end
    
    % save
    fname_cell = {'NoAdapt','Euclidean','ConstPb','Natural'};
    for expNumb=1:4
        fid = fopen(['Varied_Noise40_' fname_cell{expNumb} '.txt'], 'a');
        fprintf(fid, '%5.3f %5.3f %5.3f %5.3f %5.3f\n', RMSErrorPerRounds_sample(expNumb,:));
    end
    return;
    
%     % Tracking error plot
%     figure(104)
%     bar((1:5)',RMSErrorPerRounds_sample')
%     title('Tracking error by round (Varied sequence)')
%     xlabel('Trajectory sequence')
%     ylabel('RMS error (deg)')
%     ylim([0 5])
%     legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
%     drawnow
%     
%     % Deadzone plot
%     figure(105)
%     bar((1:5)',Deadzone')
%     title('Physically-inconsistent duration')
%     xlabel('Trajectory sequence')
%     ylabel('Time duration')
%     legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
%     drawnow
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
        ylim([0 90])
        legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
        
        figure(120+k)
        plot(tspan, rad2deg * [errorNorm_1{k}';errorNorm_2{k}';errorNorm_3{k}';errorNorm_4{k}']);
        title(['Transient tacking error: ' LoadShape_string{k}])
        xlabel('Time (sec)')
        ylabel('RMS error (deg)')
        ymax = 90;
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

%% Video
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

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% Video (Section A)
close all

color = [    0.2768    0.5415    0.7791
    0.5875    0.3907    0.4657
    0.7111    0.7612    0.7893
    0.3611    0.1208    0.5700
    0.6664    0.4437    0.2650
    0.5927    0.4170    0.3631
    0.1247    0.4426    0.7452];

% initialize G
AdaptEFonly = false;                                     %  EF only vs. Entire
WithLoad = false;
robot = wam7robot;
G = zeros(6,6,robot.nDOF);
for j =1 :robot.nDOF
    G(:,:,j) = robot.link(j).J;
end

% Make desired trajectory
TrajAmplitude = 0.8;
end_time = 10;
PeriodDesiredTraj = 5;
n_time = end_time * 10000;
tspan = linspace(0,end_time,n_time);
q_desired = zeros(robot.nDOF,n_time);
q_desired_dot = zeros(robot.nDOF,n_time);
q_desired_ddot = zeros(robot.nDOF,n_time);
for j = 1 : n_time
    [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), 0.8, robot);
end

% plots
sp = 0.05; sp2 = 0.4;
xx = 1/3; yy = 1/(1+sp2); zz = sp2/(1+sp2); ww = sp/(1+sp2); ww2 = ww*2
aa = figure(10);
set(aa, 'position',[150 150 1500 625]);
set(aa,'defaultfigurecolor',[1 1 1]);
axislim = 0.8;
height = 1.2;
TailLength = 30;
iter = 1;
for k =1 :500:  size(state_augmented_nat_entire{iter},1) % time(sec) * 10000
    % subplots

    % plot in zero position
    
    %% Natural
    % parsing
    state_augmented = state_augmented_nat_entire{iter};
    theta = state_augmented(k,1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    [T_ref, ~] = forkine(robot,q_desired(:,k)');
    if(k==1)
        EFpos_nat = T(1:3,4);
        EFpos_ref = T_ref(1:3,4);
    else
        EFpos_nat = [EFpos_nat T(1:3,4)];
        EFpos_ref = [EFpos_ref T_ref(1:3,4)];
    end
    if(size(EFpos_nat,2)>TailLength)
        EFpos_nat(:,1) = [];
        EFpos_ref(:,1) = [];
    end
    G_out = p2G(state_augmented(k,2*robot.nDOF+1:end)');
    
    % entire
    nat1 = subplot(2,3,1);
    plot_inertiatensor(Tsave, G_out, 0.7, color); hold on;
    line(EFpos_nat(1,:), EFpos_nat(2,:), EFpos_nat(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    
    % mass ratio
    nat2 = subplot(2,3,4);
    hold on; ylim([0 2]);
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    bar(y);
    xlabel('Link number');
    ylabel('m_{adapt}/m_{true}');
    ylim([0 2]);
    set(nat2, 'xtick', 1:robot.nDOF, 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
    
    %% Pullback
    % parsing
    state_augmented = state_augmented_pb_entire{iter};
    theta = state_augmented(k,1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    if(k==1)
        EFpos_pb = T(1:3,4);
    else
        EFpos_pb = [EFpos_pb T(1:3,4)];
    end
    if(size(EFpos_pb,2)>TailLength)
        EFpos_pb(:,1) = [];
    end
    G_out = p2G(state_augmented(k,2*robot.nDOF+1:end)');
    
    % entire
    pb1 = subplot(2,3,2);
    plot_inertiatensor(Tsave, G_out, 0.7, color); hold on;
    line(EFpos_pb(1,:), EFpos_pb(2,:), EFpos_pb(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    
    % mass ratio
    pb2 = subplot(2,3,5);
    hold on; ylim([0 2]);
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    bar(y);
    xlabel('Link number');
    ylabel('m_{adapt}/m_{true}');
    ylim([0 2]);
    set(pb2, 'xtick', 1:robot.nDOF, 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
    
    %% Euclidean
    % parsing
    state_augmented = state_augmented_euc_entire{iter};
    theta = state_augmented(k,1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    if(k==1)
        EFpos_euc = T(1:3,4);
    else
        EFpos_euc = [EFpos_euc T(1:3,4)];
    end
    if(size(EFpos_euc,2)>TailLength)
        EFpos_euc(:,1) = [];
    end
    G_out = p2G(state_augmented(k,2*robot.nDOF+1:end)');
    
    % entire
    euc1 = subplot(2,3,3);
    plot_inertiatensor(Tsave, G_out, 0.7, color); hold on;
    line(EFpos_euc(1,:), EFpos_euc(2,:), EFpos_euc(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    
    % mass ratio
    euc2 = subplot(2,3,6);
    hold on; ylim([0 2]);
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    bar(y);
    xlabel('Link number');
    ylabel('m_{adapt}/m_{true}');
    ylim([0 2]);
    set(euc2, 'xtick', 1:robot.nDOF, 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
    
    %% subplot position
    set(nat1,'position',[ww, zz + ww, xx - 2*ww, yy - 2*ww]);
    set(nat2,'position',[ww2, ww2, xx-2*ww2, zz - 2*ww2]);
    set(pb1,'position',[xx+ ww, zz + ww, xx - 2*ww, yy - 2*ww]);
    set(pb2,'position',[xx+ww2, ww2, xx-2*ww2, zz - 2*ww2]);
    set(euc1,'position',[2*xx+ ww, zz + ww, xx - 2*ww, yy - 2*ww]);
    set(euc2,'position',[2*xx+ww2, ww2, xx-2*ww2, zz - 2*ww2]);
    drawnow;
end

%% Adapting video (Section B)
close all

loadnum = 3;

color = [    0.2768    0.5415    0.7791
    0.5875    0.3907    0.4657
    0.7111    0.7612    0.7893
    0.3611    0.1208    0.5700
    0.6664    0.4437    0.2650
    0.5927    0.4170    0.3631
    0.1247    0.4426    0.7452];

% desired traj
end_time = 25;
n_time = end_time * 10000;
tspan = linspace(0,end_time,n_time);
q_desired = zeros(robot.nDOF,n_time);
q_desired_dot = zeros(robot.nDOF,n_time);
q_desired_ddot = zeros(robot.nDOF,n_time);
for j = 1 : n_time
    [q_desired(:,j), q_desired_dot(:,j), q_desired_ddot(:,j)] = make_desired_trajectory_sample(tspan(j), TrajAmplitude, robot);
end

% initialize G
LoadShape_cell = {0,1,2};  % 0: Sphere, 1: parallelepiped, 2:ellispoid with offset
LoadShape = LoadShape_cell{loadnum};
robot = wam7robot_with_load;
G = zeros(6,6,robot.nDOF);
for j =1 :robot.nDOF
    G(:,:,j) = robot.link(j).J;
end

stepsize = 500;
fv = cell(7,1);
fv_cur_nat = cell(7,floor(size(state_augmented_nat{loadnum},1)/stepsize)+1);
fv_cur_pb = cell(7,floor(size(state_augmented_nat{loadnum},1)/stepsize)+1);
fv_cur_euc = cell(7,floor(size(state_augmented_nat{loadnum},1)/stepsize)+1);
for i = 1 : robot.nDOF
    fv{i} = stlread([num2str(i),'.stl']);
    %     fv{i}.vertices = (Tsave(1:3,1:3,i)*fv{i}.vertices' + Tsave(1:3,4,i)*ones(1,size(fv{i}.vertices,1)))';
end
n=1;
for k = 1:stepsize:size(state_augmented_nat{loadnum},1)
    state_augmented = state_augmented_nat{loadnum};
    [T, Tsave] = forkine(robot,state_augmented(k,:));
    for i = 1 : robot.nDOF
        fv_cur_nat{i,n} = fv{i};
        fv_cur_nat{i,n}.vertices = (Tsave(1:3,1:3,i)*fv{i}.vertices' + Tsave(1:3,4,i)*ones(1,size(fv{i}.vertices,1)))';
    end
    
    state_augmented = state_augmented_pb{loadnum};
    [T, Tsave] = forkine(robot,state_augmented(k,:));
    for i = 1 : robot.nDOF
        fv_cur_pb{i,n} = fv{i};
        fv_cur_pb{i,n}.vertices = (Tsave(1:3,1:3,i)*fv{i}.vertices' + Tsave(1:3,4,i)*ones(1,size(fv{i}.vertices,1)))';
    end
    
    state_augmented = state_augmented_euc{loadnum};
    [T, Tsave] = forkine(robot,state_augmented(k,:));
    for i = 1 : robot.nDOF
        fv_cur_euc{i,n} = fv{i};
        fv_cur_euc{i,n}.vertices = (Tsave(1:3,1:3,i)*fv{i}.vertices' + Tsave(1:3,4,i)*ones(1,size(fv{i}.vertices,1)))';
    end
    n=n+1;
end

sp = 0.05; sp2 = 0.4;
xx = 1/3; yy = 1/(1+sp2); zz = sp2/(1+sp2); ww = sp/(1+sp2); ww2 = ww*2;


    % settings
aa = figure(10);
set(aa, 'position',[150 150 1500 625]);
set(aa,'defaultfigurecolor',[1 1 1]);
axislim = 0.8;

nat1 = subplot(2,3,1);
set(nat1,'Tag','231');
p_nat = cell(7,1);
for i = 1 : robot.nDOF
    p_nat{i} = patch(fv_cur_nat{i,1},'FaceColor',       color(i,:), ...
        'EdgeColor',       'none',        ...
        'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.15);
end
camlight('headlight');
material('dull');
hold on;

nat2 = subplot(2,3,4);
set(nat2,'Tag','234');


pb1 = subplot(2,3,2);
set(pb1,'Tag','232');
p_pb = cell(7,1);
for i = 1 : robot.nDOF
    p_pb{i} = patch(fv_cur_pb{i,1},'FaceColor',       color(i,:), ...
        'EdgeColor',       'none',        ...
        'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.15);
end
camlight('headlight');
material('dull');
hold on;

pb2 = subplot(2,3,5);
set(pb2,'Tag','235');

euc1 = subplot(2,3,3);
set(euc1,'Tag','233');
p_euc = cell(7,1);
for i = 1 : robot.nDOF
    p_euc{i} = patch(fv_cur_euc{i,1},'FaceColor',       color(i,:), ...
        'EdgeColor',       'none',        ...
            'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.15);
end
camlight('headlight');
material('dull');
hold on;

euc2 = subplot(2,3,6);
set(euc2,'Tag','236');

% subplot position
set(nat1,'position',[ww, zz + ww, xx - 2*ww, yy - 2*ww]);
set(nat2,'position',[ww2, ww2, xx-2*ww2, zz - 2*ww2]);
set(pb1,'position',[xx+ ww, zz + ww, xx - 2*ww, yy - 2*ww]);
set(pb2,'position',[xx+ww2, ww2, xx-2*ww2, zz - 2*ww2]);
set(euc1,'position',[2*xx+ ww, zz + ww, xx - 2*ww, yy - 2*ww]);
set(euc2,'position',[2*xx+ww2, ww2, xx-2*ww2, zz - 2*ww2]);

%
pause(0);
TailLength = 30;
axislim = 0.8;
height = 1.2;
for k = 1 : size(fv_cur_nat,2) % time(sec) * 10000

   %% Natural
    % parsing
    state_augmented = state_augmented_nat{loadnum};
    theta = state_augmented(1+stepsize*(k-1),1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    [T_ref, ~] = forkine(robot,q_desired(:,1+stepsize*(k-1))');
    if(1+stepsize*(k-1)==1)
        EFpos_nat = T(1:3,4);
        EFpos_ref = T_ref(1:3,4);
    else
        EFpos_nat = [EFpos_nat T(1:3,4)];
        EFpos_ref = [EFpos_ref T_ref(1:3,4)];
    end
    if(size(EFpos_nat,2)>TailLength)
        EFpos_nat(:,1) = [];
        EFpos_ref(:,1) = [];
    end
    G_out = p2G(state_augmented(1+stepsize*(k-1),2*robot.nDOF+1:end)');
    
    % EFonly
%     if(k==1)
%         nat1 = subplot(2,3,1);    hold on;
%         for i = 1 : robot.nDOF
%             set(p_nat{i}, 'Vertices', fv_cur_nat{i,k}.vertices);
%             % Fix the axes scaling, and set a nice view angle
%             axis([-1 1 -1 1 -1 1]);
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     else
%         h = findobj('Tag','231');
%         set(h,'NextPlot','add')
%         idxend = size(h.Children,1);
%         
%         for i = 1 : robot.nDOF
%             h.Children(idxend-7+i).Vertices = fv_cur_nat{i,k}.vertices;
%             h.Children(idxend-7+i).Faces = fv_cur_nat{i,k}.faces;
%             % Fix the axes scaling, and set a nice view angle
% %             axis([-1 1 -1 1 -1 1]);
%             h.XLim = [-1 1];
%             h.YLim = [-1 1];
%             h.ZLim = [-1 1];
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     end
    
    h = findobj('Tag','231');
%     set(h,'NextPlot','add')
    axes(h)
    hold on;
    for i = 1 : robot.nDOF
        set(p_nat{i}, 'Vertices', fv_cur_nat{i,k}.vertices);
        % Fix the axes scaling, and set a nice view angle
        
        %         draw_SE3(eye(4,4));
    end
    
    if(k~=1)
        delete(h.Children(1:end-8))
    end
    
    plot_inertiatensor(Tsave(:,:,end), G_out(:,:,end), 0.7, color(end,:)); hold on;
    line(EFpos_nat(1,:), EFpos_nat(2,:), EFpos_nat(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    hold off;
    
    % mass ratio
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    if(k==1)
        h = findobj('Tag','234');
        axes(h)
        bar_nat = bar(h, y);
        xlabel('Link number');
        ylabel('m_{adapt}/m_{true}');
        ylim([0 2]);
        h.XTick =  1:robot.nDOF;
        h.XTickLabel = {'1','2', '3', '4', '5', '6','7'}';
    else
        bar_nat.YData = y';
    end
    
   %% Pullback
    % parsing
    state_augmented = state_augmented_pb{loadnum};
    theta = state_augmented(1+stepsize*(k-1),1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    if(1+stepsize*(k-1)==1)
        EFpos_pb = T(1:3,4);
    else
        EFpos_pb = [EFpos_pb T(1:3,4)];
    end
    if(size(EFpos_pb,2)>TailLength)
        EFpos_pb(:,1) = [];
    end
    G_out = p2G(state_augmented(1+stepsize*(k-1),2*robot.nDOF+1:end)');
    
    
    % EFonly   
%     if(k==1)
%         pb1 = subplot(2,3,2);
%         hold on;
%         for i = 1 : robot.nDOF
%             set(p_pb{i}, 'Vertices', fv_cur_pb{i,k}.vertices);
%             % Fix the axes scaling, and set a nice view angle
%             axis([-1 1 -1 1 -1 1]);
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     else
%         h = findobj('Tag','232');
%         set(h,'NextPlot','add')
%         idxend = size(h.Children,1);
%         
%         for i = 1 : robot.nDOF
%             h.Children(idxend-7+i).Vertices = fv_cur_pb{i,k}.vertices;
%             h.Children(idxend-7+i).Faces = fv_cur_pb{i,k}.faces;
%             % Fix the axes scaling, and set a nice view angle
%             axis([-1 1 -1 1 -1 1]);
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     end
    
    h = findobj('Tag','232');
    axes(h)
    hold on;
    for i = 1 : robot.nDOF
            set(p_pb{i}, 'Vertices', fv_cur_pb{i,k}.vertices);
            % Fix the axes scaling, and set a nice view angle
            
            %         draw_SE3(eye(4,4));
    end
    if(k~=1)
        delete(h.Children(1:end-8))
    end
    plot_inertiatensor(Tsave(:,:,end), G_out(:,:,end), 0.7, color(end,:)); hold on;
    line(EFpos_pb(1,:), EFpos_pb(2,:), EFpos_pb(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    hold off;
    
    % mass ratio
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    if(k==1)
        h = findobj('Tag','235');
        axes(h)
        bar_pb = bar(h, y);
        xlabel('Link number');
        ylabel('m_{adapt}/m_{true}');
        ylim([0 2]);
        h.XTick =  1:robot.nDOF;
        h.XTickLabel = {'1','2', '3', '4', '5', '6','7'}';
    else
        bar_pb.YData = y';
    end
    %% Euclidean
    % parsing
    state_augmented = state_augmented_euc{loadnum};
    theta = state_augmented(1+stepsize*(k-1),1:robot.nDOF);
    [T, Tsave] = forkine(robot,theta);
    if(1+stepsize*(k-1)==1)
        EFpos_euc = T(1:3,4);
    else
        EFpos_euc = [EFpos_euc T(1:3,4)];
    end
    if(size(EFpos_euc,2)>TailLength)
        EFpos_euc(:,1) = [];
    end
    G_out = p2G(state_augmented(1+stepsize*(k-1),2*robot.nDOF+1:end)');
    
    
    % robot
%     if(k==1)
%         euc1 = subplot(2,3,3);
%         hold on;
%         for i = 1 : robot.nDOF
%             set(p_pb{i}, 'Vertices', fv_cur_euc{i,k}.vertices);
%             % Fix the axes scaling, and set a nice view angle
%             axis([-1 1 -1 1 -1 1]);
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     else
%         h = findobj('Tag','233');
%         set(h,'NextPlot','add')
%         idxend = size(h.Children,1);
%         
%         for i = 1 : robot.nDOF
%             h.Children(idxend-7+i).Vertices = fv_cur_euc{i,k}.vertices;
%             h.Children(idxend-7+i).Faces = fv_cur_euc{i,k}.faces;
%             % Fix the axes scaling, and set a nice view angle
%             axis([-1 1 -1 1 -1 1]);
%             view([-135 35]);
%             %         draw_SE3(eye(4,4));
%         end
%     end
    
    h = findobj('Tag','233');
    axes(h)
    hold on;
    for i = 1 : robot.nDOF
            set(p_euc{i}, 'Vertices', fv_cur_euc{i,k}.vertices);
            % Fix the axes scaling, and set a nice view angle
            axis([-axislim axislim -axislim axislim -0.3 1.0]);
            view([-135 35]);
            %         draw_SE3(eye(4,4));
    end
    if(k~=1)
        delete(h.Children(1:end-8))
    end   
    plot_inertiatensor(Tsave(:,:,end), G_out(:,:,end), 0.7, color(end,:)); hold on;
    line(EFpos_euc(1,:), EFpos_euc(2,:), EFpos_euc(3,:),'Color','red','LineWidth',1) %,'LineStyle','--'
    line(EFpos_ref(1,:), EFpos_ref(2,:), EFpos_ref(3,:),'Color','blue','LineWidth',1)
    draw_SE3(eye(4,4));
    for i = 1 : robot.nDOF
        draw_SE3(Tsave(:,:,i));
    end
    axis([-axislim axislim -axislim axislim -0.3 height]);
    view([-135 35]);
    hold off;
    
    % mass ratio
    y = reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1);
    if(k==1)
        h = findobj('Tag','236');
        axes(h)
        bar_euc = bar(h, y);
        xlabel('Link number');
        ylabel('m_{adapt}/m_{true}');
        ylim([0 2]);
        h.XTick =  1:robot.nDOF;
        h.XTickLabel = {'1','2', '3', '4', '5', '6','7'}';
    else
        bar_euc.YData = y';
    end

    drawnow;
    
end