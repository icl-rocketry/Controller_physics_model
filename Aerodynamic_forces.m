function [F_aero_body, tau_aero_body] = Aerodynamic_forces(state, u_fins, alpha, beta, x_cg, Tables, params)
% calculates aerodynamic forces and torques acting on the rocket in body axes
    
    % extract important values
    params.S_ref = S_ref; 
    params.L_ref = L_ref;

    % obtain mach number and dynamic pressure
    flow_V = norm(state(4:6));
    Mach_n = get_mach(state(3), flow_V);
    [rho, ~, ~] = standard_atm_function(state(3));
    q = 0.5 * rho * (flow_V ^ 2);

    % calculate aerodynamic coefficients
    [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, ...
        flow_v, Mach_n, L_ref, x_cg, S_ref, alpha, beta, u_fins, ...
        params.x_gridfin, params.S_ref_gridfin, params.R_rocket, ...
        params.chord_gridfin, Tables);
    
    % calculate aero forces
    Fx = - C_A * q * S_ref;
    Fy = C_Y * q * S_ref;
    Fz = - C_N * q * S_ref;
    
    % calculate aero torques
    tau_x = C_l * q * S_ref * L_ref;
    tau_y = C_m * q * S_ref * L_ref;
    tau_z = C_n * q * S_ref * L_ref;
    
    % forculate force and torque vectors
    F_aero_body = [Fx, Fy, Fz];
    tau_aero_body = [tau_x, tau_y, tau_z];
end