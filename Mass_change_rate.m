function [Mass_ROC] = Mass_change_rate(Thrust, Isp, g)
% Obtain mass ROC wrt time.
%   Obtains the mass flow rate of the engine with the thrust, Isp of the
%   engine and value of gravity
Mass_ROC = - Thrust / (Isp * g);
end