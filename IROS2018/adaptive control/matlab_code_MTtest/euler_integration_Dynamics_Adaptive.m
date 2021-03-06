function [t, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented)
global dt NaturalAdaptation AdaptEFonly

t = tspan;
n_time = size(tspan,2);
state_augmented = zeros(n_time, size(state0_augmented,1));
state_augmented(1,:) = state0_augmented';
StopMax = round(n_time / tspan(end) / 1000); % 1 kHz controller
Stopper = 0;

% updated_state_parameter = state0_augmented(robot.nDOF*2+1:end);
%     for i=1:robot.nDOF
%         ith_updated_S = G2S(p2G(updated_state_parameter(10*(i-1)+1:10*i,1)));
%         [~,p] = chol(ith_updated_S);
%         if(p ~= 0)
%             ith_updated_S
%             disp(i)
%         end
%     end


for i = 2 : n_time
    t_cur = tspan(i);
    
    % Controller frequency 1 kHz
    Stopper = Stopper + 1;
    if(Stopper > StopMax)
        Stopper = 1;
    end
    dot_state_augmented = Dynamics_Adaptive(t_cur, state_augmented(i-1,:)', robot, Stopper)';
    %%%%%%%%%%%%%%%%% ���� ó��%%%%%%%%%%%%%%%%%
    if(isnan(dot_state_augmented))
        state_augmented = nan;
        return;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    % Copy the previous state parameter
    state_augmented(i,robot.nDOF*2+1:end) = state_augmented(i-1,robot.nDOF*2+1:end);
    
    if(NaturalAdaptation) % natural adapatation
        % joint state
        state_augmented(i,1:robot.nDOF*2) = state_augmented(i-1,1:robot.nDOF*2) + dt * dot_state_augmented(1,1:robot.nDOF*2);
        % inertial parameter
        if(AdaptEFonly)
            % Inertial update frequency 1kHz.
            if(Stopper == 1)                
                j = robot.nDOF;
                
                P = G2S(p2G(state_augmented(i-1,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j)'));
                dP = ( dt * StopMax ) * G2S(p2G(dot_state_augmented(1,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j)'));
                sqrtmP = sqrtm(P);
                sqrtmpinvP = sqrtm(pinv(P));
                updatedP = sqrtmP * expm( sqrtmpinvP * dP * sqrtmpinvP) * sqrtm(P);
                
                state_augmented(i,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j) = G2p(S2G(updatedP))';
            end
            
        else % entire body update
            
            % Inertial update frequency 1kHz.
            if(Stopper == 1)
                for j=1:robot.nDOF
                    P = G2S(p2G(state_augmented(i-1,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j)'));
                    dP = ( dt * StopMax ) * G2S(p2G(dot_state_augmented(1,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j)'));
                    
                    sqrtmP = sqrtm(P);
                    sqrtmpinvP = sqrtm(pinv(P));
                    updatedP = sqrtmP * expm( sqrtmpinvP * dP * sqrtmpinvP) * sqrtm(P);
                    
                    state_augmented(i,robot.nDOF*2+1+10*(j-1):robot.nDOF*2+10*j) = G2p(S2G(updatedP))';
                end
            end
            
            
        end
    else % Euclidean adaptation
        % joint state
        state_augmented(i,1:robot.nDOF*2)           = state_augmented(i-1,1:robot.nDOF*2)     + dt * dot_state_augmented(1,1:robot.nDOF*2);
        % inertial parameter: freq 1 kHz
        if(Stopper == 1)
            state_augmented(i,robot.nDOF*2+1:end)   = state_augmented(i-1,robot.nDOF*2+1:end) + ( dt * StopMax ) * dot_state_augmented(robot.nDOF*2+1:end);
        end
    end
    
    %     % plot
    %     if(rem(i,20) == 1 )
    %         color = [0.3013    0.5590    0.3308
    %             0.2955    0.8541    0.8985
    %             0.3329    0.3479    0.1182
    %             0.4671    0.4460    0.9884
    %             0.6482    0.0542    0.5400
    %             0.0252    0.1771    0.7069
    %             0.8422    0.6628    0.9995];
    %         G = zeros(6,6,robot.nDOF);
    %         for j =1 :robot.nDOF
    %             G(:,:,j) = robot.link(j).J;
    %         end
    %         theta = state_augmented(i,1:robot.nDOF);
    %         [T, Tsave] = forkine(robot,theta);
    %         G_out = p2G(state_augmented(i,2*robot.nDOF+1:end)');
    %         aa = figure(10);
    %         set(aa, 'position',[150 150 1200 500]);
    %         a = subplot(1,1,1);
    %         plot_inertiatensor(Tsave, G_out, 0.7, color);hold on;
    %         draw_SE3(eye(4,4));
    %         for i = 1 : robot.nDOF
    %             draw_SE3(Tsave(:,:,i));
    %         end
    %         axis([-1.0 1.0 -1.0 1.0 -1.0 1.0]);
    %         set(a,'position',[0.1 0.1 0.35,0.8]);
    %         % hold off;
    %         b = subplot(1,2,2);
    %         hold on; ylim([-2 2]);
    %         y = [reshape(G_out(4,4,:),[],1)./reshape(G(4,4,:),[],1)];
    %         bar(y);
    %         set(b,'position',[0.55 0.1 0.35,0.8]);
    %         set(b, 'xtick', [1:robot.nDOF], 'xticklabel', {'1','2', '3', '4', '5', '6','7'});
    %         drawnow;
    %     end
    
end


end

