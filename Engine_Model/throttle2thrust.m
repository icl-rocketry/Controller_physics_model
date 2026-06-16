function [T] = throttle2thrust(u, config)
% Obatin commanded thrust from throttle
    % Use linear relation
    T = u * config.throttle_scale;
end