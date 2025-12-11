function [xd, yd, dxd, dyd, ddxd, ddyd] = ParsePath(pathWorld, t, dt)

    % Extract Path Coordinates
    pathX = pathWorld(:, 1);
    pathY = pathWorld(:, 2);
    
    % Create Parameter Vector for Original Path
    s_original = linspace(0, 1, length(pathX));
    
    % Create Parameter Vector for Desired Time Points
    s_new = linspace(0, 1, length(t));
    
    % Interpolate
    xd = pchip(s_original, pathX, s_new)';
    yd = pchip(s_original, pathY, s_new)';
    
    % Compute Derivatives
    dxd  = gradient(xd, dt);
    dyd  = gradient(yd, dt);
    ddxd = gradient(dxd, dt);
    ddyd = gradient(dyd, dt);
    
end

