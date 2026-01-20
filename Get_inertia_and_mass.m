function [current_mass, CoM, J] = Get_inertia_and_mass(m_dry, m_fuel_initial, CoM_wet, CoM_dry, J_wet, J_dry)
    % Obtains the new mass the new centre of gravity and the new inertia tensor.
    m_fuel = mass - m_dry;
    current_mass = m_dry + m_fuel;
    fuel_pct = m_fuel / m_fuel_initial;
    CoM = CoM_dry + fuel_pct .* (CoM_wet - CoM_dry);
    J = J_dry + fuel_pct .* (J_wet - J_dry);
end

