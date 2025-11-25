function y_dot = Get_state_derivative(t, y, R_BI, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, Isp, g0)
v = y(4:6);
q = y(7:10);
w = y(11:13);
m = y(14);

F_body = F_thrust_body + F_aero_body; 
tau_body = tau_thrust_body + tau_aero_body;
inv_J = J ^ -1;
g = [0; 0; -g0];

r_dot = v;
v_dot = (1 / m) * (R_BI * F_body) + g;
q_dot = Quaternion_derivative(q, w);
w_dot = inv_J * (tau_body - cross(w, J .* w));
m_dot = - norm(F_thrust_body) / (Isp * g0);

y_dot = zeros(14, 1);
y_dot(1:3) = r_dot;
y_dot(4:6) = v_dot;
y_dot(7:10) = q_dot;
y_dot(11:13) = w_dot;
y_dot(14) = m_dot;
end