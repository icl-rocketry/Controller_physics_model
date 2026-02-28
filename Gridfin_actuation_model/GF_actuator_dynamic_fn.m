function [actuator_state_dot] = GF_actuator_dynamic_fn(actuator_state, u, system_config)
% Obtains the rate of change of the gridfin state from current state

    % get torque output from the motor
    tau_m = get_motor_torque(actuator_state(1), actuator_state(2), u, );

    % get transition torque from the gears
    tau_trans = get_transition_torque(actuator_state, system_config);

    % get hinge moment of the grid_fins
    % need to figure out how to handle this!
    tau_aero = get_GF_hinge_moment();
    
    % Calculate gridfin and motor acceleration 
    theta_ddot_m = (1 / Im) * (tau_m - Cm * theta_dot_m - (tau_trans / N));
    theta_ddot_L = (1 / IL) * (tau_trans - tau_aero - CL * theta_dot_L);

    % get state derivative vector
    actuator_state_dot = zeros(4);
    actuator_state_dot(1) = actuator_state(2);

end