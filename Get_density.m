function [rho] = Get_density(altitude, table)
    % Approximates current air density with standard atmospheric model.
    rho = rho0 * exp(-(altitude / H));
end