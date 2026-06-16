function config = LoadEngineTables(config)
% Adds engine performance splines to config struct
    % Read Engine thrust data
    sweep = readtable("thrust_sweep.csv");
    
    % Add spline fit data and add it to struct
    config.alpha_fuel_data = spline(sweep.thrust, sweep.alpha_fuel);
    config.alpha_ox_mapping = spline(sweep.alpha_fuel, sweep.alpha_ox);
    config.P_c_data = spline(sweep.alpha_fuel, sweep.P_c);
    config.mdot_data = spline(sweep.alpha_fuel, sweep.mdot);
    config.thrust_data = spline(sweep.alpha_fuel, sweep.thrust);
    
    % Get throttle to thrust relationship
    max_thrust = max(sweep.thrust);
    config.throttle_scale = max_thrust / 100;
end