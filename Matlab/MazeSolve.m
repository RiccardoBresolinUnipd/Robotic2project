clear;
close all;
clc;
addpath("./MazeSolveUtils/")


%% Maze Parameters
passageWidth = 4;
wallThickness = 1;
mapSize = [50 25];
mapResolution = 5;

%% Generate Maze Map
mazeMap = mapMaze(passageWidth, wallThickness, ...
    'MapSize', mapSize, 'MapResolution', mapResolution);

%% Extract Occupancy Data
occupancyMatrixPlot = occupancyMatrix(mazeMap);
[rowCount, colCount] = size(occupancyMatrixPlot);

%% Plot and Select Start/Goal Points
pacmanColorMap = [0 0 0; 0.1294 0.1294 1];
[figureMaze, axesMaze] = PlotMaze(occupancyMatrixPlot, pacmanColorMap, ...
    'Select Start', 'Select START point');

disp('Select START point...');
[startX, startY] = ginput(1);

startRow = max(1, min(rowCount, round(startY)));
startCol = max(1, min(colCount, round(startX)));

if occupancyMatrixPlot(startRow, startCol) == 1
    error('Selected START point is a wall.');
end

plot(axesMaze, startCol, startRow, 'go', 'MarkerSize', 10, 'LineWidth', 2);

% Select Goal point
title(axesMaze, 'Select GOAL point');
disp('Select GOAL point...');

[goalX, goalY] = ginput(1);

goalRow = max(1, min(rowCount, round(goalY)));
goalCol = max(1, min(colCount, round(goalX)));

if occupancyMatrixPlot(goalRow, goalCol) == 1
    error('Selected GOAL point is a wall.');
end

plot(axesMaze, goalCol, goalRow, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

startGrid = [startRow, startCol];
goalGrid  = [goalRow, goalCol];

%% Maze Solving
[plannedPathGrid, plannedPathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid);

%% Final Maze Plot
close(figureMaze);
[figureWorld, axesWorld] = PlotMaze(occupancyMatrixPlot, pacmanColorMap, ...
    'Smoothed Path', 'Smoothed A* Path on Maze');

% Convert World Path to Grid Coordinates for Plotting
mapRes  = mazeMap.Resolution;
xLimits = mazeMap.XWorldLimits;
yLimits = mazeMap.YWorldLimits;

pathXworld = plannedPathWorldSmooth(:, 1);
pathYworld = plannedPathWorldSmooth(:, 2);

pathXgrid = (pathXworld - xLimits(1)) * mapRes + 0.5;
pathYgrid = (yLimits(2) - pathYworld) * mapRes + 0.5;

% Convert Start and Goal
startWorld = grid2world(mazeMap, startGrid);
goalWorld  = grid2world(mazeMap, goalGrid);

startXgrid = (startWorld(1) - xLimits(1)) * mapRes + 0.5;
startYgrid = (yLimits(2) - startWorld(2)) * mapRes + 0.5;

goalXgrid = (goalWorld(1) - xLimits(1)) * mapRes + 0.5;
goalYgrid = (yLimits(2) - goalWorld(2)) * mapRes + 0.5;

%% Plot Final Smoothed Dashed Yellow Path
plot(axesWorld, pathXgrid, pathYgrid, '--', 'Color', [1 1 0], 'LineWidth', 2);
plot(axesWorld, startXgrid, startYgrid, 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(axesWorld, goalXgrid, goalYgrid, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

legend(axesWorld, 'Smoothed Path', 'Start', 'Goal', 'Location', 'bestoutside');
axis(axesWorld, 'tight');
