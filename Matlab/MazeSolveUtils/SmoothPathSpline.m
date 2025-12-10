function smoothPath = SmoothPathSpline(pathWorld, resolutionFactor)

    % Remove duplicates
    diffs = diff(pathWorld, 1, 1);
    segLen = sqrt(sum(diffs.^2, 2));
    keepMask = [true; segLen > 0];
    p = pathWorld(keepMask, :);

    if size(p,1) <= 3
        smoothPath = p;
        return;
    end

    % Arc-length parameterization
    diffs2 = diff(p, 1, 1);
    segLen2 = sqrt(sum(diffs2.^2, 2));
    arc = [0; cumsum(segLen2)];
    totalLen = arc(end);

    % Interpolation points
    fineCount = max(size(p,1), round(size(p,1) * resolutionFactor));
    arcFine = linspace(0, totalLen, fineCount);

    % Shape-preserving interpolation
    xs = pchip(arc, p(:,1), arcFine);
    ys = pchip(arc, p(:,2), arcFine);

    smoothPath = [xs.', ys.'];
end
