clear
clc

%% Simulation Setup

position = [0, 0, 5000];
v_inertial = [0, 0, -150];  
q = [0, 0, 0, 0];
w_body = [0, 0, 0];  
u_fins     = [0, 0, 0];  
mass = 1000;

state = [position, v_inertial, q, w_body, mass]';

%rocket parameters
x_cg = -15; %from nose
params.S_ref = 0.0314;
params.L_ref = 30;
params.R_rocket= 2;
params.x_gridfin  = -7;
params.S_ref_GF = 0.5;
params.chord_gridfins = 0.8;

%% Calculation

%find alpha and beta, done in dynamics_fn
R_BI = quat2rotm(q);
v_body = transpose(R_BI) * v_inertial';

% calculate alpha and beta (z and y)
alpha = atan2(v_body(3), -v_body(1));
beta = atan2(v_body(2), -v_body(1));

%load tables
aerosplinefits = LoadAeroTables();
Tables = struct("aerosplinefits", aerosplinefits);
params.Tables = Tables;

%% Plot

[F, tau] = Aerodynamic_forces(state, u_fins, alpha, beta, x_cg, params);

fprintf('\nF   = [%+.2f  %+.2f  %+.2f] N\n',   F(1),   F(2),   F(3));
fprintf('tau = [%+.4f  %+.4f  %+.4f] N·m\n\n', tau(1), tau(2), tau(3));

visualise(F, tau, params, x_cg);


function visualise(F, tau, params, x_cg)

    figure('Name', 'Visualisation', 'Color', 'w', 'Position', [100, 100, 900, 500]);
    clf; 

    subplot(1, 4, [1 2 3]); 
    hold on; grid on; axis equal; view(35, 22);

    %to make scale of arrows reasonable
    maxlength = params.L_ref/2;
    scalefactor = maxlength/max(abs([F,tau]));
    
    % forces
    quiver3(x_cg,0,0, F(1)*scalefactor, 0, 0, 0, '-r', 'LineWidth', 1.5, 'DisplayName', 'F_x');
    quiver3(x_cg,0,0, 0, F(2)*scalefactor, 0, 0, '-r', 'LineWidth', 1.5, 'DisplayName', 'F_y');
    quiver3(x_cg,0,0, 0, 0, F(3)*scalefactor, 0, '-r', 'LineWidth', 1.5, 'DisplayName', 'F_z');
    
    % torques
    quiver3(x_cg,0,0, tau(1)*scalefactor, 0, 0, 0, '--b', 'LineWidth', 1.5, 'DisplayName', '\tau_x');
    quiver3(x_cg,0,0, 0, tau(2)*scalefactor, 0, 0, '--b', 'LineWidth', 1.5, 'DisplayName', '\tau_y');
    quiver3(x_cg,0,0, 0, 0, tau(3)*scalefactor, 0, '--b', 'LineWidth', 1.5, 'DisplayName', '\tau_z');
   
    [Yc, Zc, Xc] = cylinder(params.R_rocket, 50); % Generate base cylinder (50 faces for smoothness)
    X_plot = Xc * (-params.L_ref);
    surf(X_plot, Yc, Zc, 'FaceColor', [0.8 0.9 0.9], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.4, 'HandleVisibility', 'off'); % HandleVisibility hides it from the legend

    xlabel('x'); ylabel('y'); zlabel('z');
    title('Forces (Solid Red) and Torques (Dashed Blue)');
    legend('Location', 'bestoutside');
    hold off;

    %plot vector values
    subplot(1, 4, 4);
    axis off;
    
    text_data = sprintf([ ...
        '\\bfForces [N]:\\rm\n', ...
        ' F_x: %10.2f\n', ...
        ' F_y: %10.2f\n', ...
        ' F_z: %10.2f\n\n', ...
        '\\bfTorques [Nm]:\\rm\n', ...
        ' \\tau_x: %10.2f\n', ...
        ' \\tau_y: %10.2f\n', ...
        ' \\tau_z: %10.2f' ...
        ], F(1), F(2), F(3), tau(1), tau(2), tau(3));
       
    text(0, 0.5, text_data, 'FontSize', 12, 'FontName', 'FixedWidth', ...
        'VerticalAlignment', 'middle', 'Interpreter', 'tex');
end