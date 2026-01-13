function [F_aero_body, tau_aero_body] = Aerodynamic_forces(flow_V, alpha, beta)
Mach_n = get_mach(flow_V);
[C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(Mach_n, alpha, beta);
Fx = 
Fy = 
Fz = 

tau_x = 
tau_y = 
tau_z = 

F_aero_body = [Fx, Fy, Fz];
tau_aero_body = [tau_x, tau_y, tau_z];
end