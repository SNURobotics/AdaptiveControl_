function [t, state_augmented] = euler_integration_Dynamics_Adaptive(robot, tspan, state0_augmented)
global dt

t = tspan;
n_time = size(tspan,2);
state_augmented = zeros(n_time, size(state0_augmented,1));
state_augmented(1,:) = state0_augmented';
for i = 2 : n_time
    i
    t_cur = tspan(i);
    state_augmented(i,:) = state_augmented(i-1,:) + dt * Dynamics_Adaptive(t_cur, state_augmented(i-1,:)', robot)';
end

end