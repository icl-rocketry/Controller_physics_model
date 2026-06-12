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
P_steady = 500000; % target stable pressure
Pa = 100000;
tau = L_star / (c_star * Gamma ^ 2);
Ae = 6463.92 / (1000 ^ 2);
At = 1520.53 / (1000 ^ 2);

% Create expected pressure curve with approximation
% Assume instanteneous combustion
step = 0;
Pc(1) = 0;
while abs(P_steady - Pc(step + 1)) > P_tolerance
    step = step + 1;
    t = step * dt;
    Pc(step + 1) = P_steady * (1 - exp(- t / tau));
end

t_series = (0:step) * dt;
figure
plot(t_series, Pc)

% Create expected thrust relation
Kp = ((1 + (gamma - 1) / 2) * Me ^ 2) ^ (gamma / (gamma - 1));
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

figure
plot(t_series, F)