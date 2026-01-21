function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(Mach_n, alpha, beta, body_trans_table, body_mom_table, grid_fins_table)
    % Woof
    opts = detectImportOptions('B0000.xlsx');
    opts.SelectedVariableNames = {'MACH', 'ALPHADSC', 'CA', 'CN', 'CFy', 'CMx', 'CMy', 'CMz'};
    T = readtable('xpitch_B0000.xlsx', opts);

    mach_vector = T.MACH;
    alpha_vector = T.ALPHADSC;

    CA_vector = T.CA;
    CN_vector = T.CN;
    CY_vector = T.CFy;
    Cl_vector = T.CMx;
    Cm_vector = T.CMy;
    Cn_vector = T.CMz;

    F_CA = scatteredInterpolant(mach_vector,alpha_vector,CA_vector, 'linear');
    F_CN = scatteredInterpolant(mach_vector,alpha_vector,CN_vector, 'linear');
    F_CY = scatteredInterpolant(mach_vector,alpha_vector,CY_vector, 'linear');
    F_Cl = scatteredInterpolant(mach_vector,alpha_vector,Cl_vector, 'linear');
    F_Cm = scatteredInterpolant(mach_vector,alpha_vector,Cm_vector, 'linear');
    F_Cn = scatteredInterpolant(mach_vector,alpha_vector,Cn_vector, 'linear');

    C_A = F_CA(Mach_n,alpha);
    C_N = F_CN(Mach_n,alpha);
    C_Y = F_CY(Mach_n,alpha);
    C_l = F_Cl(Mach_n,alpha);
    C_m = F_Cm(Mach_n,alpha);
    C_n = F_Cn(Mach_n,alpha);
end