function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, flow_v, Mach_n, L_ref, x_cg, S_ref, alpha_rad, beta_rad, grid_fin_alpha, static_table, dynamic_table, grid_fins_table, x_gridfin, R_gridfin, chord_gridfins, aerosplinefits)
    % Woof
    alpha_deg = rad2deg(alpha_rad);
    beta_deg = rad2deg(beta_rad);

    total_alpha = sqrt(alpha_deg^2 + beta_deg^2);
    deg_ratio_alpha = alpha_deg / total_alpha;
    deg_ratio_beta = beta_deg / total_alpha;

    mach_vector = static_table.MACH;
    alpha_vector = static_table.ALPHADSC;
    CA_vector = static_table.CA;
    CN_vector = static_table.CN;
    Cm_vector = static_table.CMy;
    F_CA = scatteredInterpolant(mach_vector,alpha_vector,CA_vector, 'linear');
    F_CN = scatteredInterpolant(mach_vector,alpha_vector,CN_vector, 'linear');
    F_Cm = scatteredInterpolant(mach_vector,alpha_vector,Cm_vector, 'linear');
    Ca_total = F_CA(Mach_n,total_alpha);
    Cn_total = F_CN(Mach_n,total_alpha);
    Cmy_ref = F_Cm(Mach_n,total_alpha);

    if abs(Cn_total) < 1e-4
        Cn_safe = F_CN(Mach_n, 0.1);
        Cmy_safe = F_Cm(Mach_n, 0.1);
        x_cp = -(Cmy_safe / Cn_safe) * L_ref;
    else
        x_cp = -(Cmy_ref / Cn_total) * L_ref;
    end 

    mach_vector = dynamic_table.MACH;
    alpha_vector = dynamic_table.ALPHADSC;
    CA_vector = dynamic_table.CA;
    CN_vector = dynamic_table.CN;
    Cm_vector = dynamic_table.CMy;
    F_CA = scatteredInterpolant(mach_vector,alpha_vector,CA_vector, 'linear');
    F_CN = scatteredInterpolant(mach_vector,alpha_vector,CN_vector, 'linear');
    F_Cm = scatteredInterpolant(mach_vector,alpha_vector,Cm_vector, 'linear');
    Caq = F_CA(Mach_n,total_alpha);
    Cnq = F_CN(Mach_n,total_alpha);
    Cmq = F_Cm(Mach_n,total_alpha);
    
    if flow_v < 1e-4
        flow_v = 1.0;
    end
   
    q_rad_s = state(12);
    r_rad_s = state(13);
    q_bar = (q_rad_s * L_ref) / (2 * flow_v);
    r_bar = (r_rad_s * L_ref) / (2 * flow_v);
    rate_mag = sqrt(q_bar^2 + r_bar^2);

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

    % Obtain gridfin params
    V_B = state(4:6);
    w_B = state(11:13);

    [CF_GF, CM_GF] = grid_fin_aero_coeffs(); % add function inputs later when I clean this repo

    CA_GF = sum(CF_GF(1,:)); CY_GF = sum(CF_GF(2,:)); CN_GF = sum(CF_GF(3,:));
    Cl_GF = sum(CM_GF(1,:)); Cm_GF = sum(CM_GF(2,:)); Cn_GF = sum(CM_GF(3,:));
    
    % add gridfin contributions to coefficients
    C_A = C_A + CA_GF; % x and A aligned
    C_Y = C_Y + CY_GF; % y and Y aligned
    C_N = C_N + CN_GF; % z and N aligned

    C_l = C_l + Cl_GF;
    C_m = C_m + Cm_GF;
    C_n = C_n + Cn_GF;
end