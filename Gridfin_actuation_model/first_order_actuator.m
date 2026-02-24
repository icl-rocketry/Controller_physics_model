function [theta_new] = first_order_actuator(theta, theta_target, tau, dt, max_rate)
    % Slew-fit first order actuator model
    theta_new = theta + (dt/tau) * max(theta_target - theta, max_rate);
end

