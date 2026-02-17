function x_dot = dynamics_fn(t, x, u, params)
% calculates force and torque on the body to get the state derivative of the system
    
    % Obtains current mass, Cg and inertia tensor
    m, CoM, J = Get_inertia_and_mass(params.m_dry, params.m_fuel, params.CoM_wet, params.CoM_dry, params.J_wet, params.J_dry);

    % Ensure quaternions are on the manifold
    q = x(7:10);
    q = q / norm(q);

    % Calculate body to inertial axes rotation vector
    R_BI = quat2rotm(q);

    % Calculate aerodynamic force and torque in body axes
    u_grid_fins = u(1:3);
    % calculate alpha and beta
    alpha = 0;
    beta = 0;
    flow_V = norm(x(4:6));
    F_aero_body, tau_aero_body = Aerodynamic_forces(x, u_grid_fins, flow_V, alpha, beta, params);
    
    % Calculate force and torque from engine in body axes
    u_engine = u(4:6);
    F_thrust_body, tau_thrust_body, Isp = Engine_Model(); % need to finish

    % Caculates state derivative with forces and torques
    x_dot = Get_state_derivative(t, x, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, Isp, config.g0);
end