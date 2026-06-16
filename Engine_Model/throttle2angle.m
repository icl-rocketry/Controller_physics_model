function [alpha_fuel, alpha_ox] = throttle2angle(u, config)
% Calculates valve angle targets required to achieve desired throttle
% note that u is unsaturated as engine will cut with valve angle not throttle
    % Obtain commanded thrust from throttle
    thrust_cmd = throttle2thrust(u, config);
    
    % Evaluate splines to obtain valve opening angles
    alpha_fuel = ppval(config.alpha_fuel_data, thrust_cmd);
    alpha_ox = ppval(config.alpha_ox_mapping, alpha_fuel);
end