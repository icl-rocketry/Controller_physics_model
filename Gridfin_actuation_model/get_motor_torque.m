function [tau_m] = get_motor_torque(theta_m, theta_dot_m, u, system_config)
% Calculates the motor output torque
    
    Kp, Kd, tau_stall, omega_stall, Vmax, tau_max, PW_max, PW_neutral, PW_min, theta_max, theta_min

    % calculate the PWM to angle mapping
    m = (system_config.PW_max - system_config.PW_min) / (system_config.theta_max - system_config.theta_min);
    c = system_config.PW_neutral;

    % get motor command angle
    theta_cmd = (u - c) / m;

    % predict internal PD command
    e = theta_cmd - theta_m;
    Vcmd = system_config.Kp * e - system_config.Kd * theta_dot_m;
    Vcmd = max(-system_config.Vmax, min(system_config.Vmax, Vcmd));

    % calculate motor torque
    coeff = system_config.tau_stall / system_config.omega_no_load;
    tau_m = coeff * Vcmd - coeff * theta_dot_m;
    tau_m = max(-system_config.tau_max, min(system_config.tau_max, tau_m));
end