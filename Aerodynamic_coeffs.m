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


    %%grifins:

    V_B = state(4:6);
    w_B = state(11:13);

    %gridfin definitions (location of hinge point effectively)
    G_angle_1 = 0;
    G_angle_2 = pi/2 + (2/3)*pi;
    G_angle_3 = pi/2 + (4/3)*pi;
    r_gridfin_1 = [x_gridfin; 0; R_gridfin ];
    r_gridfin_2 = [x_gridfin; R_gridfin*cos(G_angle_2); R_gridfin*sin(G_angle_2) ];
    r_gridfin_3 = [x_gridfin; R_gridfin*cos(G_angle_3); R_gridfin*sin(G_angle_3) ];

    %V_hingeframe = T(rotation from fin 1 to n in x axis) * ( V_B + w_B * r_gridfin)
    V_H_1 = [1,0,0; 0, 1, 0 ; 0, 0 , 1] * (V_B + cross(w_B, r_gridfin_1));
    V_H_2 = [1,0,0; 0, cos(G_angle_2), sin(G_angle_2) ; 0, -sin(G_angle_2) , cos(G_angle_2)] * (V_B + cross(w_B, r_gridfin_2));
    V_H_3 = [1,0,0; 0, cos(G_angle_3), sin(G_angle_3) ; 0, -sin(G_angle_3) , cos(G_angle_3)] * (V_B + cross(w_B, r_gridfin_3));

    %apply gridfin control deflection (deflections d1 to d3 in radians)
    %(rotate in z)
    V_G_1 = [cos(d1), -sin(d1), 0; sin(d1), cos(d1), 0; 0, 0, 1] * V_H_1;
    V_G_2 = [cos(d2), -sin(d2), 0; sin(d2), cos(d2), 0; 0, 0, 1] * V_H_2;
    V_G_3 = [cos(d3), -sin(d3), 0; sin(d3), cos(d3), 0; 0, 0, 1] * V_H_3;

    %angles of attack in fin frame
    alpha_G1 = atan(V_G_1(2)/V_G_1(1)); %not 100% sure of axes definition here 
    alpha_G2 = atan(V_G_2(2)/V_G_2(1));
    alpha_G3 = atan(V_G_3(2)/V_G_3(1));

    %Coefficients(alpha, M):
    CL_G1 = ppval(aerosplinefits.CLfit, alpha_G1);
    CL_G2 = ppval(aerosplinefits.CLfit, alpha_G2);
    CL_G3 = ppval(aerosplinefits.CLfit, alpha_G3);

    CD_G1 = ppval(aerosplinefits.CDfit, alpha_G1);
    CD_G2 = ppval(aerosplinefits.CDfit, alpha_G2);
    CD_G3 = ppval(aerosplinefits.CDfit, alpha_G3);

    %redimensionalise to forcess
    h = state(3); %in m?
    h0 = 40; %based off of approximate euroc location
    h = h + h0; 
    [~,~,~,rho] = atmosisa(h);

    qinf_1 = 0.5*rho*norm(V_G_1)^2;
    L_G1 = CL_G1*(qinf_1 * chord_gridfins); %redimensionalised relatiive to chord not area, unsure
    D_G1 = CD_G1*(qinf_1 * chord_gridfins);
    
    qinf_2 = 0.5*rho*norm(V_G_2)^2;
    L_G2 = CL_G2*(qinf_2 * chord_gridfins);
    D_G2 = CD_G2*(qinf_2 * chord_gridfins);
        
    qinf_3 = 0.5*rho*norm(V_G_3)^2;
    L_G3 = CL_G3*(qinf_3 * chord_gridfins);
    D_G3 = CD_G3*(qinf_3 * chord_gridfins);

    %forcing vectors in fin frame
    F_1_G = [-L_G1*sin(alpha_G1)-D_G1*cos(alpha_G1); L_G1*cos(alpha_G1) - D_G1*sin(alpha_G1); 0];
    F_2_G = [-L_G2*sin(alpha_G2)-D_G2*cos(alpha_G2); L_G2*cos(alpha_G2) - D_G2*sin(alpha_G2); 0];
    F_3_G = [-L_G3*sin(alpha_G3)-D_G3*cos(alpha_G3); L_G3*cos(alpha_G3) - D_G3*sin(alpha_G3); 0];

    %transfer to forcing coefficients in body axes:

    %unrotate gridfin control deflection about z, to hinge frame:
    F_1_H = [cos(-d1), -sin(-d1), 0; sin(-d1), cos(-d1), 0; 0, 0, 1] * F_1_G;
    F_2_H = [cos(-d2), -sin(-d2), 0; sin(-d2), cos(-d2), 0; 0, 0, 1] * F_2_G;
    F_3_H = [cos(-d3), -sin(-d3), 0; sin(-d3), cos(-d3), 0; 0, 0, 1] * F_3_G;

    %rotate back about x axis back to gridfin 1
    F_1_B = F_1_H;
    F_2_B = [1,0,0; 0, cos(-G_angle_2), sin(-G_angle_2) ; 0, -sin(-G_angle_2) , cos(-G_angle_2)] * F_2_H;
    F_3_B = [1,0,0; 0, cos(-G_angle_3), sin(-G_angle_3) ; 0, -sin(-G_angle_3) , cos(-G_angle_2)] * F_3_H;
    %x,y,z are now aligned with rocket x,y,z
   
    %non dimensionalise force vectors relative to rocket qinf and Sref 
    CF_1_B = F_1_B./(0.5*rho*(norm(V_B)^2)*Sref);
    CF_2_B = F_2_B./(0.5*rho*(norm(V_B)^2)*Sref);
    CF_3_B = F_3_B./(0.5*rho*(norm(V_B)^2)*Sref);
    
    %force coefficients: just sum to rocket coefficents
    C_A = C_A - CF_1_B(1) - CF_2_B(1) - CF_3_B(1); %x and A antiparallel
    C_Y = C_Y + CF_1_B(2) + CF_2_B(2) + CF_3_B(2); %y and Y aligned
    C_N = C_N + CF_1_B(3) + CF_2_B(3) + CF_3_B(3); %z and N aligned

    %moments: using forcing vectors at each hinge
    C_M_allgridfins = cross(r_gridfin_1, F_1_H) + cross(r_gridfin_2, F_2_H) + cross(r_gridfin_3, F_3_H);

    C_l = C_l - C_M_allgridfins(1);
    C_m = C_m + C_M_allgridfins(2);
    C_n = C_n + C_M_allgridfins(3);

end