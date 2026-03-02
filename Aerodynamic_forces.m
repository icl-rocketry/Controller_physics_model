function [F_aero_body, tau_aero_body] = Aerodynamic_forces(state, u_fins, alpha, beta, x_cg, params)
% calculates aerodynamic forces and torques acting on the rocket in body axes
    
    % extract important values
    S_ref = params.S_ref; 
    L_ref = params.L_ref;
    x_gridfin = params.x_gridfin;
    S_ref_GF = params.S_ref_GF;
    R_rocket = params.R_rocket;
    chord_gridfins = params.chord_gridfins;
    Tables = params.Tables;

    % obtain mach number and dynamic pressure
    flow_V = norm(state(4:6));
    Mach_n = get_mach(state(3), flow_V);
    [rho, ~, ~] = standard_atm_function(state(3));
    q = 0.5 * rho * (flow_V ^ 2);

    % calculate aerodynamic coefficients
    [C_X, C_Z, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, ...
        flow_V, Mach_n, rho, L_ref, x_cg, S_ref, alpha, beta, ...
        u_fins, x_gridfin, S_ref_GF, R_rocket, chord_gridfins, Tables);

    % calculate aero forces
    Fx = C_X * q * S_ref;
    Fy = C_Y * q * S_ref;
    Fz = C_Z * q * S_ref;
    
    % calculate aero torques
    tau_x = C_l * q * S_ref * L_ref;
    tau_y = C_m * q * S_ref * L_ref;
    tau_z = C_n * q * S_ref * L_ref;
    
    % forculate force and torque vectors
    F_aero_body = [Fx, Fy, Fz];
    tau_aero_body = [tau_x, tau_y, tau_z];
end