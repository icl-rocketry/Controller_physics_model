function [x_, t_] = dynamics_step(t, x, dt, u, u_)
% function to obtain the state at the following timestep
    % RK4 integration
    K1 = Dynamics_wrapper(t, x, u);
    K2 = Dynamics_wrapper(t + dt / 2, x + dt .* K1 / 2, (0.5) * (u + u_));
    K3 = Dynamics_wrapper(t + dt / 2, x + dt .* K2 / 2, (0.5) * (u + u_));
    K4 = Dynamics_wrapper(t + dt, x + dt .* K3, u_);
    x_ = x + (dt / 6) .* (K1 + 2 * K2 + 2 * K3 + K4);
    
    % step t
    t_ = t + dt;

    % snap quaternions back onto mainfold to approximate lieset integration
    q = x_(7:10); 
    x_(7:10) = q / norm(q);
end