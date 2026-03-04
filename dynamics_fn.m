function [x_dot, x_add_dot] = dynamics_fn(t, x, x_add, u, rocket_config)
% calculates force and torque on the body to get the state derivative of the system
    % Obtain Cg and inertia tensor
    [r_cg, J] = Get_inertia_and_mass(x(14), ...
        x_add(1), x_add(2), config.r_dry, config.m_dry, config.R_ox, ...
        config.R_f, config.m_ox0, config.m_fuel0, config.L0_ox, ...
        config.L0_f, config.Base_ox, config.Base_f, config.J_dry);
    x_cg = r_cg(1);

    % Ensure quaternions are on the manifold
    q = x(7:10);
    x(7:10) = q / norm(q);
    
    % Initialise additonal state derivative vector
    x_add_dot = zeros(size(x_add));

    % grid fin actuator model to get state derivative
    for i = 1:3
        actuator_state = x_add(2 + i);
        x_add_dot_GF = GF_actuator_dynamic_fn(actuator_state, ...
            u(i), rocket_config);
        x_add_dot(2 + i) = x_add_dot_GF;
    end

    % Calculate aerodynamic force and torque in body axes
    [F_aero_body, tau_aero_body] = Aerodynamic_forces(x, ...
        u(1:3), x_cg, rocket_config);
    
    % Calculate force and torque from engine in body axes
    [F_thrust_body, tau_thrust_body, Isp] = Engine_Model(); % need to finish

    % Caculates state derivative with forces and torques
    [x_dot, x_add_dot_fuel] = Get_state_derivative(t, ...
        x, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, ...
        tau_aero_body, Isp, rocket_config.OF, rocket_config.g0);

    x_add_dot(1:2) = x_add_dot_fuel(:); 
end