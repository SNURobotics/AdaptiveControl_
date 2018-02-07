% global g
% g = 9.8;
robot = wam7robot;
color = rand(robot.nDOF,3);
%% plot in zero position
theta = zeros(robot.nDOF,1);theta(1) = pi/2;
[T, Tsave] = forkine(robot,theta);
G = zeros(6,6,robot.nDOF);
for j =1 :robot.nDOF
    G(:,:,j) = robot.link(j).J;
end
plot_inertiatensor(Tsave, G, 0.1, color); hold on;
draw_SE3(eye(4,4));
for i = 1 : robot.nDOF
    draw_SE3(Tsave(:,:,i));
end


%%
n=10;
theta = zeros(robot.nDOF,n);
for i = 1 : robot.nDOF
    theta(i,:) = linspace(0,pi/2,n);
end
for i = 1 : n
%     theta(:,i) = rand(robot.nDOF,1)*pi - pi/2;
    [T, Tsave] = forkine(robot,theta(:,i));
    G = zeros(6,6,robot.nDOF);
    for j =1 :robot.nDOF
        G(:,:,j) = robot.link(j).J;
    end
%     plot_inertiatensor(Tsave, G, 0.8, color); hold on;
%     plot_inertiatensor(Tsave,G1,0.5,color);
end

numOftraj = 1;
traj1 = load('traj1.txt');

traj = cell(numOftraj,1);
traj{1}.raw = traj1;

% fc = 2.5;
% fs = 1000;
% [b,a] = butter(3,fc/(fs/2));
% for i = 1 : numOftraj
%     for j = 1 : robot.nDOF
%         traj{i}.raw(:,j+1) = filter(b,a,traj{i}.raw(:,j+1));
%     end
%     traj{i}.raw = traj{i}.raw(1000:end-1000,:);
% end

for i =1 : numOftraj
    for j =1 :200: size(traj{i}.raw,1)
        [T, Tsave] = forkine(robot,traj{1}.raw(j,2:8));
        G = zeros(6,6,robot.nDOF);
            for k =1 :robot.nDOF
                G(:,:,k) = robot.link(k).J;
            end
        figure(1);
        plot_inertiatensor(Tsave, G, 0.5, color);  axis([-0.9 0.9 -0.9 0.9 -0.3 1]);drawnow;
    end
end
for i =1 : numOftraj
    traj{i}.q = zeros(size(traj{i}.raw,1)-4,robot.nDOF);
    traj{i}.qdot = zeros(size(traj{i}.raw,1)-4,robot.nDOF);
    traj{i}.qddot = zeros(size(traj{i}.raw,1)-4,robot.nDOF);
    traj{i}.tau = zeros(size(traj{i}.raw,1)-4,robot.nDOF);
    traj{i}.time = zeros(size(traj{i}.raw,1)-4,1);
    for j = 1 : size(traj{i}.q,1)
        traj{i}.q(j,:) = traj{i}.raw(j+2,2:8);
        traj{i}.qdot(j,:) = (traj{i}.raw(j+3,2:8) - traj{i}.raw(j+1,2:8))/(traj{i}.raw(j+3,1)-traj{i}.raw(j+1,1));
        traj{i}.qddot(j,:) = ((traj{i}.raw(j+4,2:8) - traj{i}.raw(j+2,2:8))/(traj{i}.raw(j+4,1)-traj{i}.raw(j+2,1)) - (traj{i}.raw(j+2,2:8) - traj{i}.raw(j,2:8))/(traj{i}.raw(j+2,1)-traj{i}.raw(j,1)))/(traj{i}.raw(j+3,1)-traj{i}.raw(j+1,1));
        traj{i}.tau(j,:) = traj{i}.raw(j+2,9:15);
        traj{i}.time(j,:) = traj{i}.raw(j+2,1);
    end
end
fc = 2.5;
fs = 1000;
[b,a] = butter(3,fc/(fs));
for i = 1 : numOftraj
    for j = 1 : robot.nDOF
        traj{i}.q(:,j) = filter(b,a,traj{i}.q(:,j));
        traj{i}.qdot(:,j) = filter(b,a,traj{i}.qdot(:,j));
        traj{i}.qddot(:,j) = filter(b,a,traj{i}.qddot(:,j));
        traj{i}.tau(:,j) = filter(b,a,traj{i}.tau(:,j));
    end
%     traj{i}.raw = traj{i}.raw(1000:end-1000,:);
end

tau_cal = zeros(size(traj{1}.q,1),robot.nDOF);
for i =1 : size(traj{1}.q,1)
    tau_cal(i,:) = invDyn(traj{1}.q(i,:)',traj{1}.qdot(i,:)', traj{1}.qddot(i,:)', robot)';
end

% plot(b(7*[1 : size(traj{1}.q,1)]-7+6,1));
% plot(traj{1}.tau(:,1),'r');hold on;  plot(tau_cal(:,1),'g');
% bbb = A*p0;
% plot(b(7*[1 : size(traj{1}.q,1)]-7+7,1),'b'); hold on; plot(bbb(7*[1 : size(traj{1}.q,1)]-7+7,1),'r');

A = zeros(robot.nDOF* size(traj{1}.q,1),10*robot.nDOF);
b = zeros(robot.nDOF* size(traj{1}.q,1),1);
for k =1 : size(traj{1}.q,1)
    [AA,B,B_J, bb] = regression_mat(traj{1}.q(k,:)',traj{1}.qdot(k,:)', traj{1}.qddot(k,:)',traj{1}.tau(k,:)', robot);
%     [AA,B,B_J, bb] = regression_mat(traj{1}.q(k,:)',traj{1}.qdot(k,:)', traj{1}.qddot(k,:)',tau_cal(k,:)', robot);

    A(robot.nDOF*(k-1)+1:robot.nDOF*k,:) = AA;
    b(robot.nDOF*(k-1)+1:robot.nDOF*k,:) = bb;
end

n_parts = robot.nDOF;
Gbar = zeros(6,6,robot.nDOF);
for i =1 : robot.nDOF
    Gbar(:,:,i) = robot.link(i).J;
end


%%
p0 = G2p(Gbar);

%%
% options =optimoptions(@fmincon,'Algorithm', 'interior-point','SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient', true, 'TolCon',1e-15,'TolX',1e-15,'TolFun',1e-15,'MaxFunEvals', ...
%             1000000,'MaxIter',10000,'Display','iter');
% options =optimoptions(@fmincon,'Algorithm', 'interior-point','SpecifyObjectiveGradient', true, 'TolCon',1e-10,'TolX',1e-15,'TolFun',1e-6,'MaxFunEvals', ...
%     1000000,'MaxIter',10000,'Display','iter');
% 
% MM = zeros(1,size(p0,1));
% mm = zeros(n_parts,size(p0,1));
% MM(1,10*(1:n_parts)) = 1;
% for i = 1 : n_parts
%     mm(i,10*i) = -1;
% end
% tic
% [p,fval,exitflag,output,lam_costate] = fmincon(@Obj_euclidean_norm_weighted, p0, mm, zeros(n_parts,1), MM, M, [], [], [], options);
% toc
%% LMI

% 
G_barr = zeros(4,4,n_parts);
x_bar = zeros(10*n_parts,1);
for i =1 : n_parts
    G_barr(:,:,i) = [0.5 * trace(Gbar(1:3,1:3,i))*eye(3,3) - Gbar(1:3,1:3,i), skew(Gbar(1:3,4:6,i));skew(Gbar(1:3,4:6,i))', Gbar(4,4,i)];
    x_bar(10*(i-1)+1:10*i,1) = [G_barr(4,4,i), G_barr(1:3,4,i)', G_barr(1,1,i), G_barr(2,2,i),G_barr(3,3,i),G_barr(1,2,i),G_barr(2,3,i),G_barr(3,1,i)];
end

% idx_rand = randperm(size(A,1)/7,size(A,1)/7);
% A1 = A([idx_rand*7-6,idx_rand*7-5,idx_rand*7-4,idx_rand*7-3,idx_rand*7-2,idx_rand*7-1,idx_rand*7],:);
% b1 = b([idx_rand*7-6,idx_rand*7-5,idx_rand*7-4,idx_rand*7-3,idx_rand*7-2,idx_rand*7-1,idx_rand*7],:);
W_metric = pdmetric_vec(G_barr);
% W_metric = eye(70,70);
ww = 0;
inv_W_metric = inv(W_metric);
for i =1 : size(A,1)
    ww = ww + A(i,:)*inv_W_metric*A(i,:)';
end
gamma = 1e-3 * ww;
x = inv(A'*A+gamma*W_metric)*(A'*b+gamma*W_metric*x_bar);
G_stat = zeros(4,4,n_parts);
for i = 1 : n_parts
    G_stat(4,4,i) = x(10*(i-1)+1);
    G_stat(1,4,i) = x(10*(i-1)+2); G_stat(4,1,i) = x(10*(i-1)+2);
    G_stat(2,4,i) = x(10*(i-1)+3); G_stat(4,2,i) = x(10*(i-1)+3);
    G_stat(3,4,i) = x(10*(i-1)+4); G_stat(4,3,i) = x(10*(i-1)+4);
    G_stat(1,1,i) = x(10*(i-1)+5);
    G_stat(2,2,i) = x(10*(i-1)+6);
    G_stat(3,3,i) = x(10*(i-1)+7);
    G_stat(1,2,i) = x(10*(i-1)+8); G_stat(2,1,i) = x(10*(i-1)+8);
    G_stat(2,3,i) = x(10*(i-1)+9); G_stat(3,2,i) = x(10*(i-1)+9);
    G_stat(3,1,i) = x(10*(i-1)+10); G_stat(1,3,i) = x(10*(i-1)+10);
end
G = zeros(6,6,n_parts);
for i =1 : n_parts
    G(:,:,i) = [trace(G_stat(1:3,1:3,i))*eye(3,3) - G_stat(1:3,1:3,i), skew(G_stat(1:3,4,i));skew(G_stat(1:3,4,i))', G_stat(4,4,i)*eye(3,3)];
end

figure(5);
load('color_ty.mat');
plot_inertiatensor(Tsave, G,0.7,color); view([-130,10]);

bbb = A*x;
bbbb = A*x_bar;
for i = 1 :7
    figure(i);
plot(b(7*[1 : size(traj{1}.q,1)]-7+i,1),'b'); hold on; plot(bbb(7*[1 : size(traj{1}.q,1)]-7+i,1),'r'); plot(bbbb(7*[1 : size(traj{1}.q,1)]-7+i,1),'g');
end

A_red = A(:,61:70);
b_red = b - A(:,1:60)*p0(1:60,1);

x_red = A_red\b_red;

plot_inertiatensor(eye(4,4),p2G(x_red),0.7,[1,1,1]);



% W_metric = pdmetric_vec(G_barr);
% W_metric = scalemetric_vec(G_barr);
% W = zeros(size(A,1), size(A,1));
% for i = 1 : size(A,1)
%     W(i,i) = sqrt((A(i,:)*A(i,:)')/(A(i,:)*pinv(W_metric)*A(i,:)'));
% end
% W_metric_h = W_metric^(0.5);

% W = eye(size(A,1), size(A,1));
W_metric_h = eye(10*n_parts,10*n_parts);

P = zeros(4,4,10);
P(4,4,1) = 1;
P(1,4,2) = 1; P(4,1,2) = 1;
P(2,4,3) = 1; P(4,2,3) = 1;
P(3,4,4) = 1; P(4,3,4) = 1;
P(1,1,5) = 1;
P(2,2,6) = 1;
P(3,3,7) = 1;
P(1,2,8) = 1; P(2,1,8) = 1;
P(2,3,9) = 1; P(3,2,9) = 1;
P(3,1,10) = 1; P(1,3,10) = 1;

cvx_setup;

cvx_begin sdp
% cvx_begin
cvx_precision best
variable x(10*n_parts,1)

minimize( gamma * sum_square(x-x_bar) + sum_square(A*x -b))
subject to
% sum(x(1) + x(11) + x(21) + x(31) + x(41) + x(51) + x(61)) == M
for i = 1 : n_parts
    x(10*(i-1)+1)*P(:,:,1) + x(10*(i-1)+2)*P(:,:,2) + x(10*(i-1)+3)*P(:,:,3) + x(10*(i-1)+4)*P(:,:,4) + x(10*(i-1)+5)*P(:,:,5) + x(10*(i-1)+6)*P(:,:,6) + x(10*(i-1)+7)*P(:,:,7) + x(10*(i-1)+8)*P(:,:,8) + x(10*(i-1)+9)*P(:,:,9) + x(10*(i-1)+10)*P(:,:,10) - eye(4,4)*0.0>= 0
end
cvx_end

G_stat = zeros(4,4,n_parts);
for i = 1 : n_parts
    G_stat(4,4,i) = x(10*(i-1)+1);
    G_stat(1,4,i) = x(10*(i-1)+2); G_stat(4,1,i) = x(10*(i-1)+2);
    G_stat(2,4,i) = x(10*(i-1)+3); G_stat(4,2,i) = x(10*(i-1)+3);
    G_stat(3,4,i) = x(10*(i-1)+4); G_stat(4,3,i) = x(10*(i-1)+4);
    G_stat(1,1,i) = x(10*(i-1)+5);
    G_stat(2,2,i) = x(10*(i-1)+6);
    G_stat(3,3,i) = x(10*(i-1)+7);
    G_stat(1,2,i) = x(10*(i-1)+8); G_stat(2,1,i) = x(10*(i-1)+8);
    G_stat(2,3,i) = x(10*(i-1)+9); G_stat(3,2,i) = x(10*(i-1)+9);
    G_stat(3,1,i) = x(10*(i-1)+10); G_stat(1,3,i) = x(10*(i-1)+10);
end
G = zeros(6,6,n_parts);
for i =1 : n_parts
    G(:,:,i) = [trace(G_stat(1:3,1:3,i))*eye(3,3) - G_stat(1:3,1:3,i), skew(G_stat(1:3,4,i));skew(G_stat(1:3,4,i))', G_stat(4,4,i)*eye(3,3)];
end



% cvx_setup;
% G_barr = zeros(4,4,n_parts);
% for i =1 : n_parts
%     G_barr(:,:,i) = [0.5 * trace(Gbar(1:3,1:3,i))*eye(3,3) - Gbar(1:3,1:3,i), skew(Gbar(1:3,4:6,i));skew(Gbar(1:3,4:6,i))', Gbar(4,4,i)];
% end
% cvx_begin
% variable G(4,4,n_parts) semidefinite
% J = 0;
% for i = 1 : n_parts
%     J = J + 0.5 * (1/(2*sum(n_time))) * norm(G(:,:,i)-G_barr(:,:,i),'fro');
% end
% 
% minimize( J )
% subject to
% sum(G(4,4,:)) == M
% i=1; j =30;
%         trace(G(:,:,1)*[X1{i}(:,:,1,j), y1{i}(:,1,j)/2;y1{i}(:,1,j)'/2, z1{i}(1,j)]) + trace(G(:,:,2)*[X1{i}(:,:,2,j), y1{i}(:,2,j)/2;y1{i}(:,2,j)'/2, z1{i}(2,j)]) + trace(G(:,:,3)*[X1{i}(:,:,3,j), y1{i}(:,3,j)/2;y1{i}(:,3,j)'/2, z1{i}(3,j)]) + ...
%             trace(G(:,:,4)*[X1{i}(:,:,4,j), y1{i}(:,4,j)/2;y1{i}(:,4,j)'/2, z1{i}(4,j)]) + trace(G(:,:,5)*[X1{i}(:,:,5,j), y1{i}(:,5,j)/2;y1{i}(:,5,j)'/2, z1{i}(5,j)]) + trace(G(:,:,6)*[X1{i}(:,:,6,j), y1{i}(:,6,j)/2;y1{i}(:,6,j)'/2, z1{i}(6,j)]) + ...
%             trace(G(:,:,7)*[X1{i}(:,:,7,j), y1{i}(:,7,j)/2;y1{i}(:,7,j)'/2, z1{i}(7,j)]) + trace(G(:,:,8)*[X1{i}(:,:,8,j), y1{i}(:,8,j)/2;y1{i}(:,8,j)'/2, z1{i}(8,j)]) + trace(G(:,:,9)*[X1{i}(:,:,9,j), y1{i}(:,9,j)/2;y1{i}(:,9,j)'/2, z1{i}(9,j)]) + ...
%             trace(G(:,:,10)*[X1{i}(:,:,10,j), y1{i}(:,10,j)/2;y1{i}(:,10,j)'/2, z1{i}(10,j)]) + trace(G(:,:,11)*[X1{i}(:,:,11,j), y1{i}(:,11,j)/2;y1{i}(:,11,j)'/2, z1{i}(11,j)]) + trace(G(:,:,12)*[X1{i}(:,:,12,j), y1{i}(:,12,j)/2;y1{i}(:,12,j)'/2, z1{i}(12,j)]) + ...
%             trace(G(:,:,13)*[X1{i}(:,:,13,j), y1{i}(:,13,j)/2;y1{i}(:,13,j)'/2, z1{i}(13,j)]) + trace(G(:,:,14)*[X1{i}(:,:,14,j), y1{i}(:,14,j)/2;y1{i}(:,14,j)'/2, z1{i}(14,j)]) + trace(G(:,:,15)*[X1{i}(:,:,15,j), y1{i}(:,15,j)/2;y1{i}(:,15,j)'/2, z1{i}(15,j)]) + ...
%             trace(G(:,:,16)*[X1{i}(:,:,16,j), y1{i}(:,16,j)/2;y1{i}(:,16,j)'/2, z1{i}(16,j)]) == 0  % x
% %         trace(G(:,:,1)*[X2{i}(:,:,1,j), y2{i}(:,1,j)/2;y2{i}(:,1,j)'/2, z2{i}(1,j)]) + trace(G(:,:,2)*[X2{i}(:,:,2,j), y2{i}(:,2,j)/2;y2{i}(:,2,j)'/2, z2{i}(2,j)]) + trace(G(:,:,3)*[X2{i}(:,:,3,j), y2{i}(:,3,j)/2;y2{i}(:,3,j)'/2, z2{i}(3,j)]) + ...
% %             trace(G(:,:,4)*[X2{i}(:,:,4,j), y2{i}(:,4,j)/2;y2{i}(:,4,j)'/2, z2{i}(4,j)]) + trace(G(:,:,5)*[X2{i}(:,:,5,j), y2{i}(:,5,j)/2;y2{i}(:,5,j)'/2, z2{i}(5,j)]) + trace(G(:,:,6)*[X2{i}(:,:,6,j), y2{i}(:,6,j)/2;y2{i}(:,6,j)'/2, z2{i}(6,j)]) + ...
% %             trace(G(:,:,7)*[X2{i}(:,:,7,j), y2{i}(:,7,j)/2;y2{i}(:,7,j)'/2, z2{i}(7,j)]) + trace(G(:,:,8)*[X2{i}(:,:,8,j), y2{i}(:,8,j)/2;y2{i}(:,8,j)'/2, z2{i}(8,j)]) + trace(G(:,:,9)*[X2{i}(:,:,9,j), y2{i}(:,9,j)/2;y2{i}(:,9,j)'/2, z2{i}(9,j)]) + ...
% %             trace(G(:,:,10)*[X2{i}(:,:,10,j), y2{i}(:,10,j)/2;y2{i}(:,10,j)'/2, z2{i}(10,j)]) + trace(G(:,:,11)*[X2{i}(:,:,11,j), y2{i}(:,11,j)/2;y2{i}(:,11,j)'/2, z2{i}(11,j)]) + trace(G(:,:,12)*[X2{i}(:,:,12,j), y2{i}(:,12,j)/2;y2{i}(:,12,j)'/2, z2{i}(12,j)]) + ...
% %             trace(G(:,:,13)*[X2{i}(:,:,13,j), y2{i}(:,13,j)/2;y2{i}(:,13,j)'/2, z2{i}(13,j)]) + trace(G(:,:,14)*[X2{i}(:,:,14,j), y2{i}(:,14,j)/2;y2{i}(:,14,j)'/2, z2{i}(14,j)]) + trace(G(:,:,15)*[X2{i}(:,:,15,j), y2{i}(:,15,j)/2;y2{i}(:,15,j)'/2, z2{i}(15,j)]) + ...
% %             trace(G(:,:,16)*[X2{i}(:,:,16,j), y2{i}(:,16,j)/2;y2{i}(:,16,j)'/2, z2{i}(16,j)]) == 0   % y
% cvx_end


% G_stat = G;
% G = Gbar;
% for i =1 : n_parts
%     G(:,:,i) = [trace(G_stat(1:3,1:3,i))*eye(3,3) - G_stat(1:3,1:3,i), skew(G_stat(1:3,4,i));skew(G_stat(1:3,4,i))', G_stat(4,4,i)*eye(3,3)];
% end

% [r_zmp, f_z, Ax, Ay, B] = getZMP(G, T, V, Vdot);
% [r_zmp_init, f_z, Ax, Ay, B] = getZMP(Gbar, T, V, Vdot);
% 
% plot(A*x); hold on; plot(A*x_bar);
% sum1 = 0; sum2 = 0;
% curtime = -1;
% for i = 1 : n_motion
%     for j = 1 : n_time(i)
%         sum1 = 0; sum2 = 0;curtime = curtime +2;
%         for k = 1 : n_parts
%             sum1 = sum1 + trace(G(:,:,k)* Wx{i}(:,:,k,j));
%             sum2 = sum2 + trace(Gbar(:,:,k)* Wx{i}(:,:,k,j));
%         end
%         A(curtime,:)*x
%         sum1
%         A(curtime,:)*x_bar
%         sum2
%         sum3 = r_zmp{i}(2,(curtime+1)/2)*f_z{i}((curtime+1)/2) - r_zmp_BB{i}(2,(curtime+1)/2)*f_z{i}((curtime+1)/2);
%         sum3
%     end
% end

%%
% 
% G = p2G(p);

figure(5);
load('color_ty.mat');
plot_inertiatensor(T{1}(:,:,:,1), G,0.7,color); view([-130,10]);



% for i =1 : robot.nDOF
%     figure(1);
%     hold on;
%     plot(traj{1}.time(:,1),traj{1}.q(:,i));
%     figure(2);
%     hold on;
%     plot(traj{1}.time(:,1),traj{1}.qdot(:,i));
%     figure(3);
%     hold on;
%     plot(traj{1}.time(:,1),traj{1}.qddot(:,i));
%     figure(4);
%     hold on;
%     plot(traj{1}.time(:,1),tau_cal(:,i),'r-');
%     plot(traj{1}.time(:,1),traj{1}.tau(:,i));
% end