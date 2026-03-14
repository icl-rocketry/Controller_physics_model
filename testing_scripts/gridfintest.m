% GRIDFINTEST - Standalone Script (No function header)
% Clears workspace to ensure fresh start
clear; clc; close all;

% plot
f = figure('Name', 'Grid Fin Logic Tester', 'Position', [100, 100, 1200, 700], 'Color', 'w');
ax = axes('Parent', f, 'Position', [0.35, 0.1, 0.6, 0.8]);
axis(ax, 'equal'); grid(ax, 'on'); hold(ax, 'on'); view(ax, 135, 30);
xlabel(ax, 'X (Axial)'); ylabel(ax, 'Y (Lateral)'); zlabel(ax, 'Z (Vertical)');
title(ax, 'Body Frame Forces (Green) & Moments');

% sliders
uicontrol('Style','text','Pos',[20 650 250 20],'String','Velocity Vx (m/s)','HorizontalAlignment','left');
h_Vx = uicontrol('Style','slider','Pos',[20 630 250 20],'Min',10,'Max',300,'Val',100);

uicontrol('Style','text','Pos',[20 590 250 20],'String','Roll Rate p (rad/s)','HorizontalAlignment','left');
h_P = uicontrol('Style','slider','Pos',[20 570 250 20],'Min',-100,'Max',100,'Val',0);

uicontrol('Style','text','Pos',[20 530 250 20],'String','Deflection 1 (rad)','HorizontalAlignment','left');
h_d1 = uicontrol('Style','slider','Pos',[20 510 250 20],'Min',-0.5,'Max',0.5,'Val',0);

uicontrol('Style','text','Pos',[20 470 250 20],'String','Deflection 2 (rad)','HorizontalAlignment','left');
h_d2 = uicontrol('Style','slider','Pos',[20 450 250 20],'Min',-0.5,'Max',0.5,'Val',0);

uicontrol('Style','text','Pos',[20 410 250 20],'String','Deflection 3 (rad)','HorizontalAlignment','left');
h_d3 = uicontrol('Style','slider','Pos',[20 390 250 20],'Min',-0.5,'Max',0.5,'Val',0);

% Text Output
h_out = uicontrol('Style','text','Pos',[20 20 300 350],'String','Results...','HorizontalAlignment','left', 'FontName', 'Courier');

% Grid fin data loading
aerosplinefits = LoadAeroTables();

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
    chord_gridfins = 0.03; % m
    R_gridfin = 0.1; % m
    Sref = pi * (R_gridfin ^ 2); % m^2
    Sref_gridfin = 0.1;
    x_gridfin = -1.2; % m
    x_cg = -2.7;
    Sref_rocket = 0.3;
    Lref_rocket = 5;
    
    % simulate
    V_B = [vx; 0; 0];
    w_B = [p; 0; 0];
    
    [rho, ~, ~] = standard_atm_function(3000);
    q = 0.5 * rho * norm(V_B) ^ 2;

    [CF, CM] = grid_fin_aero_coeffs(V_B, ...
        w_B, q, x_cg, chord_gridfins, Sref_gridfin, ...
        Sref_rocket, Lref_rocket, R_gridfin, x_gridfin, ...
        [d1;d2;d3], rho, aerosplinefits);

    F1 = sum(CF(1,:)); F2 = sum(CF(2,:)); F3 = sum(CF(3,:));
    M1 = sum(CM(1,:)); M2 = sum(CM(2,:)); M3 = sum(CM(3,:));

    G_angle_1 = 0.0;
    G_angle_2 = (2/3) * pi;
    G_angle_3 = (4/3) * pi;
    G_angle = [G_angle_1, G_angle_2, G_angle_3];

    r_gridfin_1 = [x_gridfin - x_cg; R_gridfin * sin(G_angle_1); R_gridfin * cos(G_angle_1)];
    r_gridfin_2 = [x_gridfin - x_cg; R_gridfin * sin(G_angle_2); R_gridfin * cos(G_angle_2)];
    r_gridfin_3 = [x_gridfin - x_cg; R_gridfin * sin(G_angle_3); R_gridfin * cos(G_angle_3)];

    % Draw Rocket Body
    [Xc, Yc, Zc] = cylinder(R_gridfin, 50);
    Zc = Zc * 3 - 2; 
    surf(ax, Zc, Yc, Xc, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    scale_fac = 0.01; 
    
    % Draw Vectors
    % Bundle position vectors into a 3x3 matrix for clean looping
    r_gridfins = [r_gridfin_1, r_gridfin_2, r_gridfin_3];
    
    plot3(ax, r_gridfin_1(1), r_gridfin_1(2), r_gridfin_1(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_1(1), r_gridfin_1(2), r_gridfin_1(3), ...
        CF(1, 1)*scale_fac, CF(2, 1)*scale_fac, CF(3, 1)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
    
    plot3(ax, r_gridfin_2(1), r_gridfin_2(2), r_gridfin_2(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_2(1), r_gridfin_2(2), r_gridfin_2(3), ...
        CF(1, 2)*scale_fac, CF(2, 2)*scale_fac, CF(3, 2)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
        
    plot3(ax, r_gridfin_3(1), r_gridfin_3(2), r_gridfin_3(3), 'ro', 'MarkerFaceColor','r');
    quiver3(ax, r_gridfin_3(1), r_gridfin_3(2), r_gridfin_3(3), ...
        CF(1, 3)*scale_fac, CF(2, 3)*scale_fac, CF(3, 3)*scale_fac, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');

    % Update Text (Also corrected to reflect proper columns)
    res_str = sprintf([...
        'INPUTS:\n' ...
        'Vx: %.1f m/s\n' ...
        'Roll: %.1f rad/s\n' ...
        'Defl: [%.2f, %.2f, %.2f]\n\n' ...
        'CALCULATED FORCES (Body):\n' ...
        'F1: [%.1f, %.1f, %.1f]\n' ...
        'F2: [%.1f, %.1f, %.1f]\n' ...
        'F3: [%.1f, %.1f, %.1f]\n\n'], ...
        vx, p, d1, d2, d3, ...
        CF(1, 1), CF(2, 1), CF(3, 1), ...
        CF(1, 2), CF(2, 2), CF(3, 2), ...
        CF(1, 3), CF(2, 3), CF(3, 3));
    
    set(data.h_out, 'String', res_str);
end