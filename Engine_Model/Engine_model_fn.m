function [F_thrust, Pc, m_dot_ox, m_dot_fuel, valve_states_dot, engine_is_lit] = Engine_model_fn(throttle, valve_states, config)
% Simulates engine thrust response to valve angle
    % Compute required valve angles for thrust
    [valve_angle_cmd, ~] = throttle2angle( ...
        throttle, config);
    
    % Compute valve opening response to actuation command
    [valve_states_dot] = valve_opening_model( ...
        valve_angle_cmd, valve_states, config);

    % Compute thrust and chamber pressure for current valve opening
    [F_thrust, Pc, m_dot_fuel, m_dot_ox, engine_is_lit] = angle2data( ...
        valve_states(1), config);
end