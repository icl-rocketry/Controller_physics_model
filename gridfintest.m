% GRIDFINTEST - Standalone Script (No function header)
% Clears workspace to ensure fresh start
clear; clc; close all;

% =========================================================================
% 1. SETUP FIGURE & UI
% =========================================================================
f = figure('Name', 'Grid Fin Logic Tester', 'Position', [100, 100, 1200, 700], 'Color', 'w');

% Create Axes
ax = axes('Parent', f, 'Position', [0.35, 0.1, 0.6, 0.8]);
axis(ax, 'equal'); grid(ax, 'on'); hold(ax, 'on'); view(ax, 135, 30);
xlabel(ax, 'X (Axial)'); ylabel(ax, 'Y (Lateral)'); zlabel(ax, 'Z (Vertical)');
title(ax, 'Body Frame Forces (Green) & Moments');

% UI Controls (Sliders)
uicontrol('Style','text','Pos',[20 650 250 20],'String','Velocity Vx (m/s)','HorizontalAlignment','left');
h_Vx = uicontrol('Style','slider','Pos',[20 630 250 20],'Min',10,'Max',300,'Val',100);

uicontrol('Style','text','Pos',[20 590 250 20],'String','Roll Rate p (rad/s)','HorizontalAlignment','left');
h_P = uicontrol('Style','slider','Pos',[20 570 250 20],'Min',-10,'Max',10,'Val',0);

uicontrol('Style','text','Pos',[20 530 250 20],'String','Deflection 1 (rad)','HorizontalAlignment','left');
h_d1 = uicontrol('Style','slider','Pos',[20 510 250 20],'Min',-0.5,'Max',0.5,'Val',0);

uicontrol('Style','text','Pos',[20 470 250 20],'String','Deflection 2 (rad)','HorizontalAlignment','left');
h_d2 = uicontrol('Style','slider','Pos',[20 450 250 20],'Min',-0.5,'Max',0.5,'Val',0);

uicontrol('Style','text','Pos',[20 410 250 20],'String','Deflection 3 (rad)','HorizontalAlignment','left');
h_d3 = uicontrol('Style','slider','Pos',[20 390 250 20],'Min',-0.5,'Max',0.5,'Val',0);

% Text Output
h_out = uicontrol('Style','text','Pos',[20 20 300 350],'String','Results...','HorizontalAlignment','left', 'FontName', 'Courier');

% =========================================================================
% 2. DATA PREPARATION
% =========================================================================
% % Create dummy spline data so your code runs
% alpha_range = linspace(-pi/2, pi/2, 100);
% CL_data = 1.5 * sin(2*alpha_range); 
% CD_data = 0.5 + 1.0 * sin(alpha_range).^2;
% aerosplinefits.CLfit = spline(alpha_range, CL_data);
% aerosplinefits.CDfit = spline(alpha_range, CD_data);
% 
% % Bundle everything into a structure to pass to the function
% data.ax = ax;
% data.h_Vx = h_Vx;
% data.h_P = h_P;
% data.h_d1 = h_d1;
% data.h_d2 = h_d2;
% data.h_d3 = h_d3;
% data.h_out = h_out;
% data.aerosplinefits = aerosplinefits;

%GRIDFINS
    CL_alpha_table = readmatrix("honeycomb-clalpha.xlsx");
    CD_alpha_table = readmatrix("honeycomb-cdalpha.xlsx");
    
    %get rid of random values at the end
    CL_alpha_table = CL_alpha_table(2:90, :);

    %1D spline fits
    %to calcuate value at a point: Clval = ppval(Clfit, alpha);
    %do not exceed +- 50 degrees aoa

    Clalphaextended = [-flip(CL_alpha_table(:,1)); CL_alpha_table(:,1)];
    Clextended = [flip(CL_alpha_table(:,2)); CL_alpha_table(:,2)];

    Cdalphaextended = [-flip(CD_alpha_table(:,1)); CD_alpha_table(:,1)];
    Cdextended = [flip(CD_alpha_table(:,2)); CD_alpha_table(:,2)];

    CLfit_spline = spline(Clalphaextended, Clextended);
    CDfit_spline = spline(Cdalphaextended, Cdextended);

 

    aerosplinefits = struct("CLfit", CLfit_spline, ...
        "CDfit", CDfit_spline);  


% Bundle all UI handles and Data into 'data' struct
data.ax = ax;
data.h_Vx = h_Vx;
data.h_P = h_P;
data.h_d1 = h_d1;
data.h_d2 = h_d2;
data.h_d3 = h_d3;
data.h_out = h_out;
data.aerosplinefits = aerosplinefits;

% Store data in the figure
guidata(f, data);

% Set Callbacks
set(h_Vx, 'Callback', @(s,e) run_simulation(f));
set(h_P,  'Callback', @(s,e) run_simulation(f));
set(h_d1, 'Callback', @(s,e) run_simulation(f));
set(h_d2, 'Callback', @(s,e) run_simulation(f));
set(h_d3, 'Callback', @(s,e) run_simulation(f));

% Initial Run
run_simulation(f);

% =========================================================================
% 3. LOCAL CALCULATION FUNCTION
% =========================================================================
function run_simulation(fig_h)
    % Retrieve all our handles and data safely
    data = guidata(fig_h);
    ax = data.ax;
    aerosplinefits = data.aerosplinefits;
    
    % Clear the plot for redraw
    cla(ax);
    
    % Get Slider Values
    vx = data.h_Vx.Value;
    p = data.h_P.Value;
    d1 = data.h_d1.Value;
    d2 = data.h_d2.Value;
    d3 = data.h_d3.Value;

    % Define Constants required by your code block
    chord_gridfins = 0.1; % m
    Sref = 0.02; % m^2
    x_gridfin = 3; % m
    R_gridfin = 0.15; % m
    x_cg = 2;
    
    % Initialize Coeffs
    C_A = 0; C_Y = 0; C_N = 0;
    C_l = 0; C_m = 0; C_n = 0;

    % ---------------------------------------------------------------------
    % START USER CODE BLOCK
    % ---------------------------------------------------------------------
    %%grifins:
    V_B = [vx;0;0];
    w_B = [p;0;0];

     %gridfin definitions (location of hinge point effectively)
    G_angle_1 = 0;
    G_angle_2 = (2/3)*pi;
    G_angle_3 = (4/3)*pi;

    r_gridfin_1 = [x_gridfin - x_cg; 0; R_gridfin ];
    r_gridfin_2 = [x_gridfin - x_cg; R_gridfin*sin(G_angle_2); R_gridfin*cos(G_angle_2) ];
    r_gridfin_3 = [x_gridfin - x_cg; R_gridfin*sin(G_angle_3); R_gridfin*cos(G_angle_3) ];

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
    h = 500; %in m?
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
    F_1_G = [L_G1*sin(alpha_G1)+D_G1*cos(alpha_G1); L_G1*cos(alpha_G1) - D_G1*sin(alpha_G1); 0];
    F_2_G = [L_G2*sin(alpha_G2)+D_G2*cos(alpha_G2); L_G2*cos(alpha_G2) - D_G2*sin(alpha_G2); 0];
    F_3_G = [L_G3*sin(alpha_G3)+D_G3*cos(alpha_G3); L_G3*cos(alpha_G3) - D_G3*sin(alpha_G3); 0];

    %transfer to forcing coefficients in body axes:

    %unrotate gridfin control deflection about z, to hinge frame:
    F_1_H = [cos(-d1), -sin(-d1), 0; sin(-d1), cos(-d1), 0; 0, 0, 1] * F_1_G;
    F_2_H = [cos(-d2), -sin(-d2), 0; sin(-d2), cos(-d2), 0; 0, 0, 1] * F_2_G;
    F_3_H = [cos(-d3), -sin(-d3), 0; sin(-d3), cos(-d3), 0; 0, 0, 1] * F_3_G;

    %rotate back about x axis back to gridfin 1
    F_1_B = F_1_H;
    F_2_B = [1,0,0; 0, cos(-G_angle_2), sin(-G_angle_2) ; 0, -sin(-G_angle_2) , cos(-G_angle_2)] * F_2_H;
    F_3_B = [1,0,0; 0, cos(-G_angle_3), sin(-G_angle_3) ; 0, -sin(-G_angle_3) , cos(-G_angle_3)] * F_3_H;
    %x,y,z are now aligned with rocket x,y,z
   
    %non dimensionalise force vectors relative to rocket qinf and Sref 
    CF_1_B = F_1_B./(0.5*rho*(norm(V_B)^2)*Sref);
    CF_2_B = F_2_B./(0.5*rho*(norm(V_B)^2)*Sref);
    CF_3_B = F_3_B./(0.5*rho*(norm(V_B)^2)*Sref);
    
    %force coefficients: just sum to rocket coefficents
    C_A = C_A + CF_1_B(1) + CF_2_B(1) + CF_3_B(1); %x and A aligned
    C_Y = C_Y + CF_1_B(2) + CF_2_B(2) + CF_3_B(2); %y and Y aligned
    C_N = C_N + CF_1_B(3) + CF_2_B(3) + CF_3_B(3); %z and N aligned

    %moments: using forcing vectors at each hinge
    C_M_allgridfins = cross(r_gridfin_1, F_1_H) + cross(r_gridfin_2, F_2_H) + cross(r_gridfin_3, F_3_H);

    C_l = C_l + C_M_allgridfins(1);
    C_m = C_m + C_M_allgridfins(2);
    C_n = C_n + C_M_allgridfins(3);
    % ---------------------------------------------------------------------
    % END USER CODE BLOCK
    % ---------------------------------------------------------------------

    % --- VISUALIZATION ---
    % Draw Rocket Body
    [Xc, Yc, Zc] = cylinder(R_gridfin*0.8, 20);
    Zc = Zc * 3 - 2; 
    surf(ax, Zc, Yc, Xc, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    scale_fac = 0.01; 
    
    % Draw Vectors
    plot3(ax, r_gridfin_1(1), r_gridfin_1(2), r_gridfin_1(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_1(1), r_gridfin_1(2), r_gridfin_1(3), ...
        F_1_B(1)*scale_fac, F_1_B(2)*scale_fac, F_1_B(3)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
    
    plot3(ax, r_gridfin_2(1), r_gridfin_2(2), r_gridfin_2(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_2(1), r_gridfin_2(2), r_gridfin_2(3), ...
        F_2_H(1)*scale_fac, F_2_H(2)*scale_fac, F_2_H(3)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
        
    plot3(ax, r_gridfin_3(1), r_gridfin_3(2), r_gridfin_3(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_3(1), r_gridfin_3(2), r_gridfin_3(3), ...
        F_3_H(1)*scale_fac, F_3_H(2)*scale_fac, F_3_H(3)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');

    % Update Text
    res_str = sprintf([...
        'INPUTS:\n' ...
        'Vx: %.1f m/s\n' ...
        'Roll: %.1f rad/s\n' ...
        'Defl: [%.2f, %.2f, %.2f]\n\n' ...
        'CALCULATED FORCES (Body):\n' ...
        'F1: [%.1f, %.1f, %.1f]\n' ...
        'F2: [%.1f, %.1f, %.1f]\n' ...
        'F3: [%.1f, %.1f, %.1f]\n\n' ...
        'ALPHA (Check for 90deg stall):\n' ...
        'a1: %.1f deg\n' ...
        'a2: %.1f deg\n' ...
        'a3: %.1f deg\n\n' ...
        'GRIDFIN FORCING'...
        'TOTAL MOMENTS:\n' ...
        'Cl (Roll): %.4f\n' ...
        'Cm (Pitch): %.4f\n' ...
        'Cn (Yaw): %.4f'], ...
        vx, p, d1, d2, d3, ...
        F_1_B(1), F_1_B(2), F_1_B(3), ...
        F_2_B(1), F_2_B(2), F_2_B(3), ...
        F_3_B(1), F_3_B(2), F_3_B(3), ...
        rad2deg(alpha_G1), rad2deg(alpha_G2), rad2deg(alpha_G3), ...
        C_l, C_m, C_n);
    
    set(data.h_out, 'String', res_str);
end