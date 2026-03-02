clear
clc

%testing parameters
position = [0, 0, 5000];
v_inertial     = [0; 0; -150];  
q = [0, -0.707, 0, 0.707];
w_body = [0; 0; 0];  
u_fins     = [0; 0; 0];      % grid-fin deflections (rad)

state = [position(:); v_inertial(:); q(:); w_body(:)]';

%find alpha and beta, done in dynamics_fn
R_BI = quat2rotm(q);
v_body = transpose(R_BI) * v_inertial ;

% calculate alpha and beta (z and y)
alpha = atan2(v_body(3), -v_body(1));
beta = atan2(v_body(2), -v_body(1));

%rocket parameters
x_cg = -15; %from nose
params = struct("S_ref", 0.0314);
%params.Sref = 0.0314;
params.L_ref         = 30;
params.R_rocket      = 2;
params.x_gridfin     = -7;
params.S_ref_GF = 0.5;
params.chord_gridfins = 0.8;

%load tables
aerosplinefits = LoadAeroTables();
Tables = struct("aerosplinefits", aerosplinefits);
params.Tables = Tables;

[F, tau] = Aerodynamic_forces(state, u_fins, alpha, beta, x_cg, params);

fprintf('\nF   = [%+.2f  %+.2f  %+.2f] N\n',   F(1),   F(2),   F(3));
fprintf('tau = [%+.4f  %+.4f  %+.4f] N·m\n\n', tau(1), tau(2), tau(3));

% plot
%visualise(F, tau, params.R_rocket, params.L_ref, x_cg)


function visualise(F, tau, R, L_ref, x_cg)
% 3-D rocket with body force and torque components plotted at the CG.

    % Ensure figures don't duplicate and clear axes if re-running
    fig = findobj('type','figure','name','Aero Forces');
    if isempty(fig)
        figure('Name','Aero Forces','Color',[0.08 0.08 0.12], ...
            'NumberTitle','off','Units','normalized','Position',[0.1 0.1 0.75 0.78]);
    else
        figure(fig); clf;
    end
    
    L  = L_ref * 14;   % display length
    Ln = L * 0.2;      % nose length

    ax = axes('Color',[0.06 0.06 0.10],'GridColor',[0.3 0.3 0.4],'GridAlpha',0.5, ...
        'XColor',[0.7 0.7 0.8],'YColor',[0.7 0.7 0.8],'ZColor',[0.7 0.7 0.8]);
    grid on; hold on; axis equal; view(35,22);
    
    xlabel('x (axial/nose)','Color',[0.8 0.8 0.9]);
    ylabel('y (lateral)',   'Color',[0.8 0.8 0.9]);
    zlabel('z (normal)',    'Color',[0.8 0.8 0.9]);
    title(sprintf('|F| = %.1f N    |τ| = %.3f N·m', norm(F), norm(tau)), ...
        'Color','w','FontSize',12);

    % --- 1. SIMPLIFIED GEOMETRY ---
    [Y, Z, X] = cylinder(R, 40);
    surf(X * L, Y, Z, 'FaceColor',[0.5 0.52 0.62], 'EdgeColor','none','FaceAlpha',0.55);
    
    [Yn, Zn, Xn] = cylinder([R, 0], 12);
    surf(Xn * Ln + L, Yn, Zn, 'FaceColor',[0.75 0.5 0.5], 'EdgeColor','none','FaceAlpha',0.65);

    % --- 2. COORDINATE MAPPING & SCALING ---
    % Map negative x_cg (down from nose base) to plot coordinates
    cg_pos = [L + x_cg, 0, 0]; 
    
    Fm = norm(F);
    tm = norm(tau);
    sc_F = (L * 0.4) / max(Fm, 1e-3); 
    sc_T = (L * 0.3) / max(tm, 1e-3);

    % --- 3. PLOT FORCES ---
    plot_arrow(cg_pos, [F(1), 0, 0], sc_F, [1 0.5 0.5], '--', 'Fx');
    plot_arrow(cg_pos, [0, F(2), 0], sc_F, [0.5 1 0.5], '--', 'Fy');
    plot_arrow(cg_pos, [0, 0, F(3)], sc_F, [0.5 0.7 1], '--', 'Fz');
    plot_arrow(cg_pos, F, sc_F, [0.2 1 0.4], '-', 'Resultant F');

    % --- 4. PLOT TORQUES ---
    plot_arc(cg_pos, [tau(1), 0, 0], sc_T, [1 0.5 0.5], '--', 'τx');
    plot_arc(cg_pos, [0, tau(2), 0], sc_T, [0.5 1 0.5], '--', 'τy');
    plot_arc(cg_pos, [0, 0, tau(3)], sc_T, [0.5 0.7 1], '--', 'τz');
    plot_arc(cg_pos, tau, sc_T, [1 0.4 1], '-', 'Resultant τ');

    % --- 5. BODY-AXIS TRIAD AT NOSE TIP ---
    O = [L + Ln + L*0.06, 0, 0]; s2 = L * 0.15;
    plot_arrow(O, [s2, 0, 0], 1, 'r', '-', ''); text(O(1)+s2*1.1,O(2),O(3),'x','Color','r','FontSize',12,'FontWeight','bold');
    plot_arrow(O, [0, s2, 0], 1, 'g', '-', ''); text(O(1),O(2)+s2*1.1,O(3),'y','Color','g','FontSize',12,'FontWeight','bold');
    plot_arrow(O, [0, 0, s2], 1, 'b', '-', ''); text(O(1),O(2),O(3)+s2*1.1,'z','Color','b','FontSize',12,'FontWeight','bold');

    legend('TextColor','w','Color',[0.08 0.08 0.12],'EdgeColor',[0.3 0.3 0.4], ...
        'Location','northwest','FontSize',10,'AutoUpdate','off');
    hold off;
end

% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function plot_arrow(origin, vec, scale, color, style, name)
    if norm(vec) < 1e-6; return; end
    v = vec * scale;
    lw = 1.5; if strcmp(style, '-'); lw = 3; end % Make resultants thicker
    quiver3(origin(1), origin(2), origin(3), v(1), v(2), v(3), ...
        0, style, 'Color', color, 'LineWidth', lw, 'MaxHeadSize', 0.5, 'DisplayName', name);
end

function plot_arc(origin, vec, scale, color, style, name)
    mag = norm(vec);
    if mag < 1e-6; return; end
    
    Tn = vec / mag;
    if abs(Tn(1)) < 0.9
        p1 = cross(Tn', [1 0 0]); 
    else
        p1 = cross(Tn', [0 1 0]); 
    end
    p1 = p1 / norm(p1); 
    p2 = cross(Tn', p1);
    
    t = linspace(0, 1.4*pi, 40);
    r_a = mag * scale;
    arc = r_a * (cos(t)'*p1 + sin(t)'*p2) + origin;
    
    lw = 1.5; if strcmp(style, '-'); lw = 2.5; end
    plot3(arc(:,1), arc(:,2), arc(:,3), style, 'Color', color, 'LineWidth', lw, 'DisplayName', name);
    
    % Arc Arrowhead
    d = r_a * (-sin(t(end))*p1 + cos(t(end))*p2); 
    d = d / norm(d) * r_a * 0.3;
    ep = arc(end,:);
    quiver3(ep(1)-d(1), ep(2)-d(2), ep(3)-d(3), d(1), d(2), d(3), ...
        0, 'Color', color, 'LineWidth', lw, 'MaxHeadSize', 1, 'HandleVisibility', 'off');
end