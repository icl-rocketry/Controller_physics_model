function [F_thrust_body, tau_thrust_body, Pc, m_dot_ox, m_dot_fuel, valve_states_dot, engine_is_lit] = Engine_dynamics_fn( ...
    throttle, gimbal_angle_y, gimbal_angle_z, r_cg, r_engine, valve_states, Engine_phase, t_relight, config)
% Obtains the thrust and torque due to the engine from the rockets inertial frame.
    % Obtain engine thrust output and define gimbled angle
    if Engine_phase == 3
        % Check if flameout has occurred
        [F_thrust, Pc, m_dot_ox, m_dot_fuel, valve_states_dot, engine_is_lit] = Engine_model_fn(throttle, ...
            valve_states, config);
    elseif Engine_phase == 2
        [F_thrust, Pc, m_dot_ox, m_dot_fuel, engine_is_lit] = relight_thrust_model(t_relight, config);
        valve_states_dot = [0; 0];
    else
        F_thrust = 0;
        Pc = 0; % wont make a difference but change to atmospheric
        m_dot_ox = 0;
        m_dot_fuel = 0;
        valve_states_dot = [0; 0];
        engine_is_lit = false;
    end

    % Get total gimbal angle
    total_gimbal = sqrt((gimbal_angle_y ^ 2) + (gimbal_angle_z ^ 2));
    
    % Define force vecotrs from gimble angles
    Fx = F_thrust * sin(gimbal_angle_y);
    Fy = F_thrust * sin(gimbal_angle_z);
    Fz = F_thrust * cos(total_gimbal);
    
    % Create force and torque vectors created by engine
    F_thrust_body = [Fx; Fy; Fz];
    tau_thrust_body = cross((r_engine - r_cg), F_thrust_body);
end