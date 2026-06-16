function [thrust, P_c, mdot_fuel, mdot_ox, Engine_is_lit] = angle2data(alpha, config)
% Maximum operational thrust as recommended by prop team
    % Check valve angle is in valid range otherwise cut engine
    if (alpha < config.min_alpha_fuel) || (alpha > config.max_alpha_fuel)
        Engine_is_lit = false;
    end
    
    % Calculate returns for engine after checking if engine has cut
    if Engine_is_lit
        P_c = ppval(config.P_c, alpha);
        thrust = ppval(config.thrust, alpha);
        mdot = ppval(config.mdot, alpha);
        mdot_fuel = mdot / (1 + config.OF);
        mdot_ox = mdot * config.OF / (config.OF + 1);
    else
        thrust = 0; % will briefly be coldflow when engine cuts but its negligable
        P_c = 0; % should be atmospheric but doesnt make a difference
        mdot_fuel = 0; % m dot values should be cold flow values but need to talk this over
        mdot_ox = 0;
    end
end