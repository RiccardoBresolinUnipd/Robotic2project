clear;
close all;
clc;
addpath("./MazeSolveUtils/")

% Maze Configuration Parameters
mazeProps.passageWidth  = 4;
mazeProps.wallThickness = 1;
mazeProps.mapSize       = [50 25];
mazeProps.mapResolution = 5;

% Create Maze and Interactively Select Start/Goal Points
[mazeMap, occupancyMatrixPlot, startGrid, goalGrid, figureMaze, axesMaze] = ...
    CreateMazeAndSelectPoints(mazeProps);

% Solve Maze Using A* Algorithm
[pathGrid, plannedPathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid);

close(figureMaze);

% Plot Maze with Solved Path
pacmanColorMap = [0 0 0; 0.1294 0.1294 1];
[figureWorld, axesWorld] = PlotMaze(occupancyMatrixPlot, pacmanColorMap, ...
    'Smoothed Path', 'Smoothed A* Path on Maze');

% Convert World Coordinates to Grid Coordinates for Plotting
mapRes  = mazeMap.Resolution;
xLimits = mazeMap.XWorldLimits;
yLimits = mazeMap.YWorldLimits;

pathXworld = plannedPathWorldSmooth(:, 1);
pathYworld = plannedPathWorldSmooth(:, 2);

pathXgrid = (pathXworld - xLimits(1)) * mapRes + 0.5;
pathYgrid = (yLimits(2) - pathYworld) * mapRes + 0.5;

startWorld = grid2world(mazeMap, startGrid);
goalWorld  = grid2world(mazeMap, goalGrid);

startXgrid = (startWorld(1) - xLimits(1)) * mapRes + 0.5;
startYgrid = (yLimits(2) - startWorld(2)) * mapRes + 0.5;

goalXgrid = (goalWorld(1) - xLimits(1)) * mapRes + 0.5;
goalYgrid = (yLimits(2) - goalWorld(2)) * mapRes + 0.5;

% Plot Path, Start, and Goal Markers
plot(axesWorld, pathXgrid, pathYgrid, '--', 'Color', [1 1 0], 'LineWidth', 2);
plot(axesWorld, startXgrid, startYgrid, 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(axesWorld, goalXgrid, goalYgrid, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

legend(axesWorld, 'Smoothed Path', 'Start', 'Goal', 'Location', 'bestoutside');
axis(axesWorld, 'tight');
