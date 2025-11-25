function y_dot = Dynamics_wrapper(t, y, control_signal)
m, CoM, J = Get_inertia_and_mass(config.m_dry, config.m_fuel, config.CoM_wet, config.CoM_dry, config.J_wet, config.J_dry);
q = y(7:10);
q = q / norm(q); 
R_BI = quat2rotm(q);

F_thrust_body, tau_thrust_body, Isp = Engine_Model(); 
F_aero_body, tau_aero_body = Aero_Model();
y_dot = Get_state_derivative(t, y, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, Isp, config.g0);
end