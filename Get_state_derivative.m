function x_dot = Get_state_derivative(t, x, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, Isp, g0)
% Calculates state derivative vector given resultant forces, torques, mass and inertial moment
    
    % decompose state vector
    v = x(4:6);
    q = x(7:10);
    w = x(11:13);
    m = x(14);
    
    % get total body force and torque
    F_body = F_thrust_body + F_aero_body; 
    tau_body = tau_thrust_body + tau_aero_body;

    % invert J and get gravitational vector
    inv_J = J ^ -1;
    g = [0; 0; -g0];
     
    % get state derivatives in inertial axes
    r_dot = v;
    v_dot = (1 / m) * (R_BI * F_body) + g; % F = ma but body forces must be transformed with rotation matrix to move to inertial frame
    q_dot = Quaternion_derivative(q, w); % quaternion derivative taken in body reference frame. Note: this is equivilant to inertial frame w.o the need for transform.
    w_dot = inv_J * (tau_body - cross(w, J .* w));
    m_dot = - norm(F_thrust_body) / (Isp * g0);
    
    % create and assemble derivative vector
    x_dot = zeros(14, 1);
    x_dot(1:3) = r_dot;
    x_dot(4:6) = v_dot;
    x_dot(7:10) = q_dot;
    x_dot(11:13) = w_dot;
    x_dot(14) = m_dot;
end