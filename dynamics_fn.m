function x_dot = dynamics_fn(t, x, u)
% calculates force and torque on the body to get the state derivative of the system
    
    % Obtains current mass, Cg and inertia tensor
    m, CoM, J = Get_inertia_and_mass(config.m_dry, config.m_fuel, config.CoM_wet, config.CoM_dry, config.J_wet, config.J_dry);

    % Ensure quaternions are on the manifold
    q = x(7:10);
    q = q / norm(q); 

    % Calculate body to inertial axes rotation vector
    R_BI = quat2rotm(q);
    
    % Calculate force and torque from engine in body axes
    u_engine = u(1:3);
    F_thrust_body, tau_thrust_body, Isp = Engine_Model(); % need to finish

    % Calculate aerodynamic force and torque in body axes
    u_grid_fins = u(4:6);
    F_aero_body, tau_aero_body = Aero_Model(); % need to finish

    % Caculates state derivative with forces and torques
    x_dot = Get_state_derivative(t, x, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, Isp, config.g0);
end