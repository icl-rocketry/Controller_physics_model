function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, flow_v, Mach_n, rho, L_ref, x_cg, S_ref, alpha_rad, beta_rad, u_gridfin, x_gridfin, S_ref_GF, R_rocket, chord_gridfins, Tables)
    % Calculates aerodynamics coefficients from current state of the oncoming flow and rocket geormetry

    % convert pitch and SS angle to degrees for tables
    alpha_deg = rad2deg(alpha_rad);
    beta_deg = rad2deg(beta_rad);
    
    % obtain total AoA (axisymetric body) and the decomposition ratios
    total_alpha = sqrt(alpha_deg^2 + beta_deg^2);
    deg_ratio_alpha = alpha_deg / total_alpha;
    deg_ratio_beta = beta_deg / total_alpha;
    
    % extract static force and moment coefficients
    Ca_total = ppval(aerosplinefits.CA_static, Mach_n, alpha_total * 180/pi);
    Cn_total = ppval(aerosplinefits.CN_static, Mach_n, alpha_total * 180/pi);
    Cmy_ref = ppval(aerosplinefits.CM_pitch_static, Mach_n, alpha_total * 180/pi);
    
    % calculate centre of pressure with moment and force ratio
    if abs(Cn_total) < 1e-4
        % if normal force is too small to calculate use approximation since Cp stays constant at small AoA
        Cn_safe = ppval(aerosplinefits.CN_static, Mach_n, 0.1);
        Cmy_safe = ppval(aerosplinefits.CN_static, Mach_n, 0.1);
        x_cp = -(Cmy_safe / Cn_safe) * L_ref;
    else
        x_cp = -(Cmy_ref / Cn_total) * L_ref;
    end 
    
    % extract dynamic effect coefficients for forces and moments
    Caq = ppval(aerosplinefits.CA_dynamic, Mach_n, alpha_total * 180/pi);
    Cnq = ppval(aerosplinefits.CN_dynamic, Mach_n, alpha_total * 180/pi);
    Cmq = ppval(aerosplinefits.CM_pitch_dynamic, Mach_n, alpha_total * 180/pi);
    
    % appromimate a stationary rocket aero effects as at a slightly faster speed for numerical stability
    if flow_v < 1e-4
        flow_v = 0.1;
    end
   
    % Calculate rotational velocity magnitude to partially redimensionalise dynamic coeffs
    q_rad_s = state(12);
    r_rad_s = state(13);
    q_bar = (q_rad_s * L_ref) / (2 * flow_v);
    r_bar = (r_rad_s * L_ref) / (2 * flow_v);
    rate_mag = sqrt(q_bar^2 + r_bar^2);
    
    % calculate lever arm ratio
    lever_arm_ratio = (x_cp - x_cg) / L_ref;
    
    % CA (Cx): Axial Force
    Cx_static = Ca_total;
    Cx_dynamic = Caq * rate_mag;
    C_A = Cx_static + Cx_dynamic;

    % CN (Cz): Normal Force
    Cz_static = - Cn_total * deg_ratio_alpha;
    Cz_dynamic = - Cnq * q_bar;
    C_N = Cz_static + Cz_dynamic;

    % CY (Cy): Yaw Force
    Cy_static = - Cn_total * deg_ratio_beta;
    Cy_dynamic = - Cnq * r_bar;
    C_Y = Cy_static + Cy_dynamic;

    % Cl: Roll Torque
    % Roll torque is zero till I add actuator torque
    C_l = 0.0;
    
    % Cm: Pitch Torque
    Cm_damping = Cmq * q_bar;
    Cm_static = Cn_total * lever_arm_ratio * deg_ratio_alpha;
    C_m = Cm_static + Cm_damping;

    % Cn: Yaw Torque
    Cn_damping = Cmq * r_bar;
    Cn_static = - Cn_total * lever_arm_ratio * deg_ratio_beta;
    C_n = Cn_static + Cn_damping;

    % get body velocities for grid fin calculation
    V_B = state(4:6);
    w_B = state(11:13);
    
    % obtain gridfin aero contributions factoring in actuation angle
    [CF_GF, CM_GF] = grid_fin_aero_coeffs(V_B, ...
        w_B, x_cg, chord_gridfins, S_ref_GF, S_ref, L_ref, ...
        R_rocket, x_gridfin, u_gridfin, rho, Tables.aerosplinefits);
    
    % sum the 3 separate gridfin contributions
    CA_GF = sum(CF_GF(1,:)); CY_GF = sum(CF_GF(2,:)); CN_GF = sum(CF_GF(3,:));
    Cl_GF = sum(CM_GF(1,:)); Cm_GF = sum(CM_GF(2,:)); Cn_GF = sum(CM_GF(3,:));
    
    % add gridfin contributions to total aero coeffs
    % forces
    C_A = C_A + CA_GF; % x and A aligned
    C_Y = C_Y + CY_GF; % y and Y aligned
    C_N = C_N + CN_GF; % z and N aligned
    % moments
    C_l = C_l + Cl_GF;
    C_m = C_m + Cm_GF;
    C_n = C_n + Cn_GF;
end