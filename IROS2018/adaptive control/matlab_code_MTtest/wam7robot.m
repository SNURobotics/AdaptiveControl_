function robot = wam7robot(Tbase, includeMotor)
if nargin < 2
    includeMotor = false;
    if nargin < 1
        Tbase = eye(4);
    else
        Tbase = checkSE3(Tbase);
    end
end
%% robot kinematics, inertial parameters
robot = struct;
robot.nDOF = 7;
robot.base = Tbase;
%%%%%%%%%%%%% sub structure of robot
% base: SE3 from global frame to robot base frame
% nDOF: number of degree of freedom of robot
% link(i): {i}th link
% joints: joints of robot
% Tendeffector: SE(3) from last link frame to end-effector (or tool) frame
% (same as link(end).Ttool)
% motors: motors of robot
%%%%%%%%%%%%% sub structure of robot.link(i)
% Tci: SE3 from center of mass to {i}th link frame
% Jc: generalized inertia expressed in {i}th link's center of mass frame 
% J: generalized inertia expressed in {i}th link frame
% M: initial SE(3) from {i-1}th link frame to {i}th link frame
% screw: screw of {i}th joint expressed in {i}th link frame
% Ttool: SE(3) from {i}th link frame to tool frame of {i}th link

% Dynamic Parameters (par is 10X6 matrix)
par = [0.294863503
0.007950225
9.31E-05
0.113500168
0.000187103
0.25065343
-0.047746296
1.312477649
-0.007159328
10.76768767
0.0260684
1.35E-05
0.000117001
0.014722017
-3.66E-05
0.019348137
-0.009182943
0.120340603
0.059755955
3.87493756
0.13671601
0.016804342
-5.10E-06
0.005883535
5.29E-06
0.139513702
-0.068952728
0.37398727
5.96E-05
1.80228141
0.057192689
-1.47E-05
-8.19E-05
0.05716471
9.42E-05
0.003004404
0.011965126
-0.000550647
0.31854219
2.40016804
5.59E-05
-2.56E-07
1.88E-09
7.82E-05
8.33E-07
6.59E-05
1.10E-05
0.000632683
0.000539377
0.12376019
0.000931067
-1.48E-06
2.01E-06
0.000498334
0.000221618
0.000574835
-5.13E-05
-0.007118902
0.010316994
0.41797364
3.85E-05
1.91E-07
-1.77E-08
3.88E-05
3.62E-08
7.41E-05
-5.47E-06
1.12E-05
-0.00022211
0.06864753];
par = reshape(par,[],robot.nDOF);
m = par(10,:);
mx = par(7:9,:);
cm = zeros(3,robot.nDOF);
for num = 1:robot.nDOF
    cm(:,num) = mx(:,num)/m(num);
end

inr = zeros(3,3,robot.nDOF);
inr_info = par(1:6,:)';
for num = 1:robot.nDOF
    inr(:,:,num) = [inr_info(num,1) inr_info(num,2) inr_info(num,3);
                    inr_info(num,2) inr_info(num,4) inr_info(num,5);
                    inr_info(num,3) inr_info(num,5) inr_info(num,6);];
end

robot.srlib_inertia = zeros(robot.nDOF,10);
for i =1 : robot.nDOF
    robot.srlib_inertia(i,:) = [ inr(1,1,i), inr(2,2,i), inr(3,3,i), inr(1,2,i), inr(1,3,i), inr(2,3,i), mx(:,i)', m(i)];
end


% Kinematic Parameters
% len = [0.504 0.170 0.780 0.140 0.760 0.125];
dh_par = [-pi/2, 0, 0, 0;
    pi/2, 0, 0, 0;
    -pi/2, 0.045, 0.55,0;
    pi/2, -0.045, 0, 0;
    -pi/2, 0, 0.3, 0;
    pi/2, 0, 0, 0;
    0, 0, 0.06, 0];
for num = 1:robot.nDOF
    robot.link(num).Tci = [eye(3),-cm(:,num);zeros(1,3),1]; % SE(3) from i-th com to i-th link
    robot.link(num).Tic = [eye(3),cm(:,num);zeros(1,3),1];  % SE(3) from i-th link to i-th com
    robot.link(num).J = [inr(:,:,num), skew(mx(:,num)); -skew(mx(:,num)), m(num)*eye(3)]; % generalized inertia expressed in i-th link
%     robot.link(num).Jc = Adj(robot.link(num).Tic)'*robot.link(num).J*Adj(robot.link(num).Tic);  % generalized inertia expressed in i-th com
    robot.link(num).M = [RotZ(dh_par(num,4)),[0;0;dh_par(num,3)];0,0,0,1]*[RotX(dh_par(num,1)),[dh_par(num,2);0;0];0,0,0,1];
    robot.link(num).screw = Adj(invSE3(robot.link(num).M))*[0;0;1;0;0;0]; % z-axis
    robot.link(num).Ttool = [eye(3),[0;0;0];zeros(1,3),1];
end

% robot.link(1).M = [eye(3),[0;0;len(1)];zeros(1,3),1];
% robot.link(2).M = [RotX(-pi/2)*RotZ(-pi/2),[len(2);0;0];zeros(1,3),1];
% robot.link(3).M = [eye(3),[len(3);0;0];zeros(1,3),1];
% robot.link(4).M = [RotX(-pi/2),[len(4);len(5);0];zeros(1,3),1];
% robot.link(5).M = [RotX(pi/2),[0;0;0];zeros(1,3),1];
% robot.link(6).M = [RotX(-pi/2),[0;0;0];zeros(1,3),1];
% robot.link(7).M = [];

robot.Tendeffector = [eye(3),[0;0;0];zeros(1,3),1];
robot.link(robot.nDOF).Ttool = robot.Tendeffector;

%% Robot joint parameters
%%%%%%%%%%%%% sub structure of robot.joints
% qmin: min joint value
% qmax: max joint value
% dqmax: max joint velocity
% ddqmax: max joint acceleration
% dddqmax: max joint jerk
% taumax: max torque

% kv: viscous friction coefficient of joints
% (should be multiplied with joint value)
% kc: Coulomb friction coefficient of joints 
% (should be multiplied with joint value)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% joint constraints (column vector)
robot.joints.qmax = [175;90;70;180;135;360]*pi/180;
robot.joints.qmin = -[175;100;145;180;135;360]*pi/180;
robot.joints.dqmax = [100;80;140;290;290;440]*pi/180;   % user defined
% robot.joints.dqmax = [165;165;165;360;360;600]*pi/180;   % robot maximum
robot.joints.ddqmax = 5*robot.joints.dqmax;             % user defined
robot.joints.dddqmax = 300*ones(robot.nDOF,1);          % user defined
robot.joints.taumax = 3000*ones(robot.nDOF,1);            % user defined

% joint friction parameters
robot.joints.kv = [1.63; 2.372;0.8554;0.6153;0.1457;0.1437;0.05763];
robot.joints.kc = [1.601;0.533 ;0.3454;0.6291;0.03597;0.06428;0.0118];
robot.joints.ko = [0.01037;0.898;-0.09566;0.05353;0.004681;-0.007354;-0.002192];

%% Robot motor parameters
%%%%%%%%%%%%% sub structure of robot.motors
% smax: motor max speed
% imax: motor max current

% gearRatio: gear ratio of motors w.r.t joint (nDOF by 1 vector)
% (gearRatio*RotJoint = RotMotor, gearRatio*MotorTorque = JointTorque)
% L: inductance of motors (unit: H) (nDOF by 1 vector)
% R: resistance of motors (unit: Ohm) (nDOF by 1 vector)
% kt: torque constant of motors (unit: Nm/A) (nDOF by 1 vector)
% kb: back EMF constant of motors (unit: V*s or Nm/A) (nDOF by 1 vector)
% (same as kt in ideal case)
% J: rotor inertia of motors (unit: kg*m^2) (nDOF by 1 vector)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if includeMotor
    % motor constraints (column vector)
    robot.motors.smax = [3000;3000;4500;4500;4500;4500]*2*pi/60;    % user defined
%     robot.motors.smax = [5000;5000;5000;5000;5000;50000]*2*pi/60;    % motor maximum
    robot.motors.imax = [39;39;20;12;12;7.2];
    
    % motor parameters
    robot.motors.gearRatio = [147;153;153;76.95;80;51];
    robot.motors.L = [3.5;3.5;5.2;8;8;8]*1e-3;
    robot.motors.R = [0.58;0.58;0.8;2.9;2.9;7.5];
    robot.motors.kt = [0.73;0.73;0.5;0.4;0.4;0.39];
    robot.motors.kb = robot.motors.kt;
    robot.motors.J = [1.06;1.06;0.13;0.044;0.044;0.027]*1e-3;
end