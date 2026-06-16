clear;
clc;

% Setup
dt = 0.0001; % 20 ms is hopefully time we run algorithm at
P_tolerance = 10;
gamma = 1.2;
Gamma = sqrt(gamma * (2 / (gamma + 1)) ^ ((gamma + 1) / (gamma - 1)));
R = 287;
Me = 2.75;
L_star = 0.75;
c_star_eff = 0.959;
c_star_ideal = 987;
c_star = c_star_eff * c_star_ideal;
P_steady = 400000; % target stable pressure
P_tank = 1600000;
Pa = 100000;
Ae = 6463.92 / (1000 ^ 2);
At = 1520.53 / (1000 ^ 2);
OF = 1.4;

valve_nf = 100;
valve_dr = 1.0;

% Create expected pressure curve with approximation
% Assume instanteneous combustion
C_thermo = (c_star^2) * (Gamma^2) / (L_star * At);
K_actual = (P_steady * At / c_star) / sqrt(P_tank - P_steady);
Pc(1) = 0;
mdot_ox(1) = 0;
mdot_fuel(1) = 0;
xv(1) = 0;
xv_dot(1) = 0;
step = 1;

while Pc(step) < (P_steady - 100)
    xv_ddot = (valve_nf ^ 2) * (1.0 - xv(step)) - 2 * valve_dr * valve_nf * xv_dot(step);
    xv_dot(step + 1) = xv_dot(step) + xv_ddot * dt;
    xv(step + 1) = xv(step) + xv_dot(step) * dt;

    mdot_in = K_actual * xv(step) * sqrt(max(0, P_tank - Pc(step)));
    mdot_out = Pc(step) * At / c_star;
    dPcdt = C_thermo * (mdot_in - mdot_out);
    Pc(step + 1) = Pc(step) + dPcdt * dt;
    
    mdot_fuel(step) = mdot_in / (1 + OF);
    mdot_ox(step) = mdot_in / (1 + (1 / OF));

    step = step + 1;
end

mdot_in(step) = K_actual * xv(step) * sqrt(max(0, P_tank - Pc(step)));
mdot_fuel(step) = mdot_in(step) / (1 + OF);
mdot_ox(step) = mdot_in(step) / (1 + (1 / OF));

% Create expected thrust relation
Kp = (1 + ((gamma - 1) / 2) * Me ^ 2) ^ (gamma / (gamma - 1));
crit_ratio = ((gamma + 1) / 2) ^ (gamma / (gamma - 1));
Pc_choke = 0.4 * Pa * crit_ratio;

Pc_threshold = 0.4 * Pa * Kp;
F(1) = 0;
for i = 1:length(Pc)
    if Pc(i) < Pc_choke
        F(i) = 0;
    elseif Pc(i) < Pc_threshold % separated
        A_sep = At * Gamma / (((0.4 * Pa / Pc(i)) ^ (1 / gamma)) * sqrt((2 * gamma / (gamma - 1)) * (1 - (0.4 * Pa / Pc(i)) ^ ((gamma - 1) / gamma))));
        F(i) = Gamma * At * Pc(i) * (sqrt((2 * gamma / (gamma - 1)) * (1 - (0.4 * Pa / Pc(i)) ^ ((gamma - 1) / gamma)))) + (0.4 * Pa - Pa) * A_sep;
    else
        F(i) = Gamma * At * Pc(i) * (sqrt((2 * gamma / (gamma - 1)) * (1 - (1 / Kp) ^ ((gamma - 1) / gamma)))) + ((Pc(i) / Kp) - Pa) * Ae;
    end
end

% Create Time series
t_series = (0:step-1) * dt;

% Export data
save_q = input("Save the data? (0 = no, 1 = yes) ");
if save_q
    var_names = ["t" "T" "Pc" "mdot_fuel" "mdot_ox" "xv" "xv_dot"];
    export_table = table(t_series', F', Pc', mdot_fuel', mdot_ox', xv', xv_dot', VariableNames=var_names);
    writetable(export_table, "relight_data.csv")
end

figure
plot(t_series, Pc)

figure
hold on
plot(t_series, mdot_ox)
plot(t_series, mdot_fuel)
hold off

figure
plot(t_series, F)