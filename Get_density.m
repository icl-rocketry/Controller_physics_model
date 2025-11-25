function [rho] = Get_density(altitude, rho0, H)
% Approximates current air density with standard atmospheric model.
rho = rho0 * exp(-(altitude / H));
end