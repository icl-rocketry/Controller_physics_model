function [x_, x_add_, t_] = dynamics_step(t, x, x_add, dt, u, config)
% function to obtain the state at the following timestep
    % RK4 integration for state
    K1 = dynamics_fn(t, x, x_add, u, config);
    K2 = dynamics_fn(t + dt / 2, x + dt .* K1 / 2, x_add, u, config);
    K3 = dynamics_fn(t + dt / 2, x + dt .* K2 / 2, x_add, u, config);
    K4 = dynamics_fn(t + dt, x + dt .* K3, x_add, u, config);
    x_ = x + (dt / 6) .* (K1 + 2 * K2 + 2 * K3 + K4);

    % RK4 integration for additional states
    K2 = dynamics_fn(t + dt / 2, x, x_add + dt .* K1 / 2, u, config);
    K3 = dynamics_fn(t + dt / 2, x, x_add + dt .* K2 / 2, u, config);
    K4 = dynamics_fn(t + dt, x, x_add + dt .* K3, u, config);
    x_add_ = x_add + (dt / 6) .* (K1 + 2 * K2 + 2 * K3 + K4);
    
    % step t
    t_ = t + dt;

    % snap quaternions back onto mainfold to approximate lieset integration
    q = x_(7:10); 
    x_(7:10) = q / norm(q);
end