function [F_aero_body, tau_aero_body] = Aerodynamic_forces(flow_V, alpha, beta, S_ref, L_ref)
    Mach_n = get_mach(flow_V);
    q = 0.5 * rho * (flow_V^2);
    [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(Mach_n, alpha, beta);

    Fx = - C_A * q * S_ref;
    Fy = C_Y * q * S_ref;
    Fz = - C_N * q * S_ref;
    
    tau_x = C_l * q * S_ref * L_ref;
    tau_y = C_m * q * S_ref * L_ref;
    tau_z = C_n * q * S_ref * L_ref;
    
    F_aero_body = [Fx, Fy, Fz];
    tau_aero_body = [tau_x, tau_y, tau_z];
end