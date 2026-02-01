function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, flow_v, Mach_n, L_ref, x_cg, alpha_rad, beta_rad, grid_fin_alpha, static_table, dynamic_table, grid_fins_table)
% Caculates aero coefficients for forces and torques acting on the body with lookup tables
    
    % converts angle of attack from degrees to radians
    alpha_deg = rad2deg(alpha_rad);
    beta_deg = rad2deg(beta_rad);
    
    % calculate total AoA and AoA ratio components in each axis
    total_alpha = sqrt(alpha_deg^2 + beta_deg^2);
    deg_ratio_alpha = alpha_deg / total_alpha;
    deg_ratio_beta = beta_deg / total_alpha;
    
    % Interpolate to get static forces and torques (will be changes to splines)
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
    
    % Calculate centre of pressure using relation between normal force and normal torque
    if abs(Cn_total) < 1e-4
        % if normal force is to small for numerical safety use a larger value to prevent division by zero (safe approximation as Cp is
        % usually stagnant at lower AoA where this would occur
        Cn_safe = F_CN(Mach_n, 0.1);
        Cmy_safe = F_Cm(Mach_n, 0.1);
        x_cp = -(Cmy_safe / Cn_safe) * L_ref;
    else
        % if Cn is large enough calculate Cp normally
        x_cp = -(Cmy_ref / Cn_total) * L_ref;
    end 
    
    % interpolate to obtain dynamic coefficients (will be replaced by splines)
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
    
    % for numerical safety if flow velocity is to small scale it up for dynamics
    if flow_v < 1e-4
        flow_v = 1.0;
    end
   
    % obtain rotational rates
    q_rad_s = state(12);
    r_rad_s = state(13);

    % normalise rotational rates and obtain total rate
    q_bar = (q_rad_s * L_ref) / (2 * flow_v);
    r_bar = (r_rad_s * L_ref) / (2 * flow_v);
    rate_mag = sqrt(q_bar^2 + r_bar^2);
    
    % find normalised leverage of aero forces around Cg
    lever_arm_ratio = (x_cp - x_cg) / L_ref;

    % Grid fin calculations (WIP)
    
    % get local gridfin alpha 
    V_B = state(1:3)';
    w_B = state(12:14)';

    % gridfin definitions
    x_gridfin = 3;
    R_gridfin = 0.5;
    r_gridfin_1 = [x_gridfin; 0; R ];
    r_gridfin_2 = [x_gridfin; R_gridfin*cos((2/3)*pi); R_gridfin*sin((2/3)*pi) ];
    r_gridfin_3 = [x_gridfin; R_gridfin*cos((4/3)*pi); R_gridfin*sin((4/3)*pi) ];

    % V_G_gridfin = T(GB) * ( V_B + w_B * r_gridfin)
    V_G_1 = [1,0,0; 0, 1, 0 ; 0, 0 , 1] * (V_B + cross(w_B, r_gridfin_1));
    V_G_2 = [1,0,0; 0, cos((2/3)*pi), sin((2/3)*pi) ; 0, -sin((2/3)*pi) , cos((2/3)*pi)] * (V_B + cross(w_B, r_gridfin_2));
    V_G_3 = [1,0,0; 0, cos((4/3)*pi), sin((4/3)*pi) ; 0, -sin((4/3)*pi) , cos((2/3)*pi)] * (V_B + cross(w_B, r_gridfin_3));

    alpha_G1 = atan(V_G_1(3)/V_G_1(1));
    alpha_G2 = atan(V_G_2(3)/V_G_2(1));
    alpha_G3 = atan(V_G_3(3)/V_G_3(1));
    
    
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
end