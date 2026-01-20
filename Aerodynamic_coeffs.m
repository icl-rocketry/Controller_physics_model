function [C_A, C_N, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(Mach_n, alpha, beta, body_trans_table, body_mom_table, grid_fins_table)
    % Woof
    % Needs to: Ouput something that isnt zero
    % Interpolate from the table B0000 for translational coeffs and xpitch_B0000 for torque coeffs
    C_A = 0;
    C_N = 0;
    C_Y = 0;
    C_l = 0;
    C_m = 0;
    
    C_n = 0;
end