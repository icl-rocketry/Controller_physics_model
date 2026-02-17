function [x_, t_] = dynamics_step(t, x, dt, u)
% function to obtain the state at the following timestep
    % RK4 integration
    K1 = dynamics_fn(t, x, u);
    K2 = dynamics_fn(t + dt / 2, x + dt .* K1 / 2, u);
    K3 = dynamics_fn(t + dt / 2, x + dt .* K2 / 2, u);
    K4 = dynamics_fn(t + dt, x + dt .* K3, u);
    x_ = x + (dt / 6) .* (K1 + 2 * K2 + 2 * K3 + K4);
    
    % step t
    t_ = t + dt;

    % snap quaternions back onto mainfold to approximate lieset integration
    q = x_(7:10); 
    x_(7:10) = q / norm(q);
end