function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, flow_v, Mach_n, alpha_rad, beta_rad, grid_fin_alpha, static_table, dynamic_table, grid_fins_table)
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
    Ca_total = F_CA(Mach_n,alpha_total);
    Cn_total = F_CN(Mach_n,alpha_total);
    Cmy_ref = F_Cm(Mach_n,alpha_total);

    if abs(Cn_total) < 1e-4
        Cn_safe = F_CN(Mach_n, 0.1);
        Cmy_safe = F_Cm(Mach_n, 0.1);
        x_cp = -(Cmy_safe / Cn_safe) * L_ref;
    else
        x_cp = -(Cmy_ref / Cn_total) * L_ref;
    end 
    
    % Static
    C_A_static = Ca_total;
    C_Y_static = - Cn_total * deg_ratio_beta;
    C_N_static = - Cn_total * ratio_alpha;

    C_m_static = Cm_static_mag * deg_ratio_alpha;
    C_n_static = Cm_static_mag * deg_ratio_beta;

    lever_arm = (x_cg - x_cp) / L_ref;

    % Dynamic
    mach_vector = dynamic_table.MACH;
    alpha_vector = dynamic_table.ALPHADSC;
    CA_vector = dynamic_table.CA;
    CN_vector = dynamic_table.CN;
    Cm_vector = dynamic_table.CMy;
    F_CA = scatteredInterpolant(mach_vector,alpha_vector,CA_vector, 'linear');
    F_CN = scatteredInterpolant(mach_vector,alpha_vector,CN_vector, 'linear');
    F_Cm = scatteredInterpolant(mach_vector,alpha_vector,Cm_vector, 'linear');
    Caq = F_CA(Mach_n,alpha_total);
    Cnq = F_CN(Mach_n,alpha_total);
    Cmq = F_Cm(Mach_n,alpha_total);
    
    if flow_v < 1e-4
        flow_v = 1.0;
    end 
   
    q_rad_s = state(12);
    r_rad_s = state(13);
    q_bar = (q_rad_s * L_ref) / (2 * flow_v);
    r_bar = (r_rad_s * L_ref) / (2 * flow_v);
    rate_mag = sqrt(q_bar^2 + r_bar^2);

    Cm_damping = Cmq * q_bar;
    Cn_damping = Cmq * r_bar;
    
    C_l = 0.0; % will be non-zero with addition of fins
    C_m = C_m_static + Cm_damping;
    C_n = C_n_static + Cn_damping;
end