function [thrust, Pc, m_dot_ox, m_dot_fuel, valve_state, engine_is_lit] = relight_thrust_model(t_relight, config)
% calculates the output thrust of the engine during relight phase
    % Get Pressure value from pressure curve
    Pc = ppval(config.relight_Pc_data, t_relight);

    % Get thrust from thrust curve
    thrust = ppval(config.relight_thrust_data, t_relight);

    % Get mass flow rate from mdot curve
    m_dot_ox = ppval(config.relight_mdot_ox_data, t_relight);
    m_dot_fuel = ppval(config.relight_mdot_fuel_data, t_relight);

    % Get valve state
    alpha = ppval(config.relight_valve_state_data, t_relight);

    % Get valve state derivative
    alpha_dot = ppval(config.relight_valve_state_derivative_data, t_relight);

    % Construct valve state
    valve_state = [alpha; alpha_dot];
    
    % Check if relight has been achieved
    if Pc >= config.Pc_target
        engine_is_lit = true;
    else
        engine_is_lit = false;
    end
end