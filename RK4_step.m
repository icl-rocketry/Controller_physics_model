function [y_, t_] = RK4_step(t, y, dt, control_signal)
    K1 = Dynamics_wrapper(t, y, control_signal);
    K2 = Dynamics_wrapper(t + dt / 2, y + dt .* K1 / 2, control_signal);
    K3 = Dynamics_wrapper(t + dt / 2, y + dt .* K2 / 2, control_signal);
    K4 = Dynamics_wrapper(t + dt, y + dt .* K3, control_signal);
    
    y_ = y + (dt / 6) .* (K1 + 2 * K2 + 2 * K3 + K4);
    q = y_(7:10); 
    y_(7:10) = q / norm(q);
    t_ = t + dt;
end