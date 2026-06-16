function [F_thrust, Pc, m_dot_ox, m_dot_fuel, valve_state, engine_is_lit] = relight_thrust_model(t_relight, config)
% calculates the output thrust of the engine during relight phase
    % Get Pressure value from pressure curve

    % Get thrust from thrust curve

    % Get valve position from curve

    % Get mass flow rate from mdot curve
    
    % Check if relight has been achieved
    if Pc >= config.Pc_target
        engine_is_lit = true;
    else
        engine_is_lit = false;
    end
end