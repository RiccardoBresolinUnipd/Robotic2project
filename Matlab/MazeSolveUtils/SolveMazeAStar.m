function [pathGrid, pathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid)

    % Extract occupancy matrix
    occMatrix = occupancyMatrix(mazeMap);
    freeMask = ~occMatrix;

    % Clearance map
    distanceCells = bwdist(occMatrix == 1);
    minClearance = 2;
    clearanceMask = freeMask & (distanceCells >= minClearance);

    if ~any(clearanceMask(:))
        error('Clearance removed all free space.');
    end

    % Adjust start/goal if needed
    [freeRows, freeCols] = find(clearanceMask);

    sr = startGrid(1); sc = startGrid(2);
    gr = goalGrid(1); gc = goalGrid(2);

    if ~clearanceMask(sr, sc)
        d = (freeRows - sr).^2 + (freeCols - sc).^2;
        [~, idx] = min(d);
        startGrid = [freeRows(idx), freeCols(idx)];
    end

    if ~clearanceMask(gr, gc)
        d = (freeRows - gr).^2 + (freeCols - gc).^2;
        [~, idx] = min(d);
        goalGrid = [freeRows(idx), freeCols(idx)];
    end

    % Planner map
    planningMatrix = ~clearanceMask;
    planningMap = binaryOccupancyMap(planningMatrix, mazeMap.Resolution);

    % A* search
    planner = plannerAStarGrid(planningMap);
    pathGrid = plan(planner, startGrid, goalGrid);

    if isempty(pathGrid)
        error('A* failed to find a path.');
    end

    % Convert to world coordinates
    pathWorld = grid2world(mazeMap, pathGrid);

    % Smooth path
    pathWorldSmooth = SmoothPathSpline(pathWorld, 5);
end
