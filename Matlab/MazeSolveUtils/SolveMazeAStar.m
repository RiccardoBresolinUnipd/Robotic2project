function [pathGrid, pathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid)
    % Solves Maze Using A* Algorithm with Clearance Constraints

    % Extract Occupancy Matrix
    occMatrix = occupancyMatrix(mazeMap);
    freeMask = ~occMatrix;

    % Clearance Map Ensures Minimum Distance from Walls
    distanceCells = bwdist(occMatrix == 1);
    minClearance = 2;
    clearanceMask = freeMask & (distanceCells >= minClearance);

    if ~any(clearanceMask(:))
        error('Clearance removed all free space.');
    end

    % Adjust Start/Goal if Needed, Move to Nearest Valid Point if Invalid
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

    % Planner Map
    planningMatrix = ~clearanceMask;
    planningMap = binaryOccupancyMap(planningMatrix, mazeMap.Resolution);

    % A* Search Finds Path from Start to Goal
    planner = plannerAStarGrid(planningMap);
    pathGrid = plan(planner, startGrid, goalGrid);

    if isempty(pathGrid)
        error('A* failed to find a path.');
    end

    % Convert to World Coordinates
    pathWorld = grid2world(mazeMap, pathGrid);

    % Smooth Path
    pathWorldSmooth = SmoothPathSpline(pathWorld, 5);
end
