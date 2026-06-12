function [F_thrust, Pc, m_dot_ox, m_dot_fuel, valve_states_dot, engine_is_lit] = Engine_model_fn(throttle, valve_states, engine_is_lit, config)
% Simulates engine thrust response to valve angle
    if (valve_states(1) < config.min_valve_angle_fuel) && (valve_states(2) < config.min_valve_angle_ox)
        engine_is_lit = false;
    end
 
    % Compute required valve angles for thrust
    [valve_angle_fuel_cmd, valve_angle_ox_cmd] = get_valve_angle_targets_from_throttle( ...
        throttle, config);
    
    % Compute valve opening response to actuation command
    [valve_states_dot] = valve_opening_model( ...
        valve_angle_fuel_cmd, valve_angle_ox_cmd, valve_states, config);

    % Compute thrust and chamber pressure for current valve opening
    if engine_is_lit
        [F_thrust, Pc, m_dot_fuel, m_dot_ox] = get_thrust_from_valves( ...
            valve_states, config);
    else
        F_thrust = 0;
        Pc = 0;
        m_dot_fuel = 0; % m dot values should be cold flow values but need to talk this over
        m_dot_ox = 0;
    end
end