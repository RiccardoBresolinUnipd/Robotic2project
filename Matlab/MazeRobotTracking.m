clear
close all
clc
addpath("./MazeSolveUtils/")
addpath("./ControlUtils/")

%% Parameters
dt = 0.01;
T  = 15;            % time
t  = 0:dt:T;

% Controller
linear = true;      % true = linear controller  - false = non-linear controller

%% Maze Setup
% Maze Configuration Parameters
mazeProps.passageWidth  = 4;
mazeProps.wallThickness = 1;
mazeProps.mapSize       = [15 15];
mazeProps.mapResolution = 5;

% Create Maze and Interactively Select Start/Goal Points
[mazeMap, occupancyMatrixPlot, startGrid, goalGrid, figureMaze, axesMaze] = ...
    CreateMazeAndSelectPoints(mazeProps);

% Solve Maze Using A* Algorithm
[pathGrid, plannedPathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid);

close(figureMaze);

%% Parse Path
% Parse Path to Match Simulation Time Points
[xd, yd, dxd, dyd, ddxd, ddyd] = ParsePath(plannedPathWorldSmooth, t, dt);

% Get Starting Position from Path
x_curr = xd(1);
y_curr = yd(1);
theta_curr = atan2(dyd(1), dxd(1));

%% Simulation Setup
x = zeros(length(t),1);
y = zeros(length(t),1);
theta = zeros(length(t),1);

%% Visualization Setup
fig = figure('Name', 'Robot Tracking', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);

% Subplot 1: Maze (Left Side - Spans Both Rows)
axesMaze = subplot(2, 2, [1, 3]);
pacmanColorMap = [0 0 0; 0.1294 0.1294 1];

% Plot Maze on Subplot
[rowCount, colCount] = size(occupancyMatrixPlot);
imagesc(axesMaze, occupancyMatrixPlot);
axis(axesMaze, 'equal', 'tight');
set(axesMaze, 'YDir', 'normal');
colormap(axesMaze, pacmanColorMap);
set(axesMaze, 'Color', 'k');
set(axesMaze, ...
    'XTick', [], ...
    'YTick', [], ...
    'XTickLabel', [], ...
    'YTickLabel', []);
title(axesMaze, 'Robot Tracking on Maze');
hold(axesMaze, 'on');

% Convert World Coordinates to Grid Coordinates for Plotting
mapRes  = mazeMap.Resolution;
xLimits = mazeMap.XWorldLimits;
yLimits = mazeMap.YWorldLimits;

% Convert Path to Grid Coordinates
pathXgrid = (xd - xLimits(1)) * mapRes + 0.5;
pathYgrid = (yLimits(2) - yd) * mapRes + 0.5;

% Convert Start/Goal to Grid Coordinates
startWorld = grid2world(mazeMap, startGrid);
goalWorld  = grid2world(mazeMap, goalGrid);

startXgrid = (startWorld(1) - xLimits(1)) * mapRes + 0.5;
startYgrid = (yLimits(2) - startWorld(2)) * mapRes + 0.5;

goalXgrid = (goalWorld(1) - xLimits(1)) * mapRes + 0.5;
goalYgrid = (yLimits(2) - goalWorld(2)) * mapRes + 0.5;

% Plot Path, Start, and Goal on Maze
plot(axesMaze, pathXgrid, pathYgrid, 'r--', 'LineWidth', 1.5);
plot(axesMaze, startXgrid, startYgrid, 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(axesMaze, goalXgrid, goalYgrid, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Initialize Robot Plot (Yellow Dot - Like Pacman)
robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
robotPlot = plot(axesMaze, robotXgrid, robotYgrid, 'yo', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'y', 'LineWidth', 2);

% Initialize AnimatedLine for Robot Traveled Path (Yellow)
robotPathLine = animatedline(axesMaze, 'Color', 'y', 'LineWidth', 2);
addpoints(robotPathLine, robotXgrid, robotYgrid);

legend(axesMaze, 'Desired Path', 'Start', 'Goal', 'Robot', 'Traveled Path', 'Location', 'bestoutside');
axis(axesMaze, 'tight');

% Subplot 2: X and Y Positions (Top Right)
axesPos = subplot(2, 2, 2);
hold(axesPos, 'on');
grid(axesPos, 'on');

% X Position - Desired and Actual
xDesiredPlot = plot(axesPos, t, xd, 'r--', 'LineWidth', 1.5);
xActualPlot = plot(axesPos, t(1), x_curr, 'r-', 'LineWidth', 2);

% Y Position - Desired and Actual
yDesiredPlot = plot(axesPos, t, yd, 'b--', 'LineWidth', 1.5);
yActualPlot = plot(axesPos, t(1), y_curr, 'b-', 'LineWidth', 2);
xlabel(axesPos, 'Time (s)');
ylabel(axesPos, 'Position (m)');
title(axesPos, 'X and Y Position Trajectories');
legend(axesPos, 'X Desired', 'X Actual', 'Y Desired', 'Y Actual', 'Location', 'nw');
xlim(axesPos, [0, T]);

% Subplot 4: X and Y Tracking Errors (Bottom Right)
axesError = subplot(2, 2, 4);
hold(axesError, 'on');
grid(axesError, 'on');

% Zero reference line
plot(axesError, [0, T], [0, 0], 'k--', 'LineWidth', 1);

% Initialize error plots
xErrorPlot = plot(axesError, t(1), xd(1) - x_curr, 'r-', 'LineWidth', 2);
yErrorPlot = plot(axesError, t(1), yd(1) - y_curr, 'b-', 'LineWidth', 2);
xlabel(axesError, 'Time (s)');
ylabel(axesError, 'Tracking Error (m)');
title(axesError, 'X and Y Tracking Errors');
legend(axesError, 'Zero Reference', 'X Error', 'Y Error', 'Location', 'nw');
xlim(axesError, [0, T]);
ylim(axesError, [-0.35, 0.35]);

%% Simulation
for k = 1:length(t)

    % State
    x(k) = x_curr;
    y(k) = y_curr;
    theta(k) = theta_curr;

    % Desired Trajectory
    x_d   = xd(k);
    y_d   = yd(k);
    dx_d  = dxd(k);
    dy_d  = dyd(k);
    ddx_d = ddxd(k);
    ddy_d = ddyd(k);

    % Differential Flatness
    [x_d, y_d, theta_d, vd, wd] = differential_flatness(x_d, y_d, dx_d, dy_d, ddx_d, ddy_d);

    % Tracking Controller
    [v, w] = tracking_controller(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, vd, wd, linear);

    % Unicycle Model
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

    % Update Robot Visualization
    robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
    robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
    set(robotPlot, 'XData', robotXgrid, 'YData', robotYgrid);

    % Update Robot Traveled Path
    addpoints(robotPathLine, robotXgrid, robotYgrid);

    % Update X and Y Position Plots
    set(xActualPlot, 'XData', t(1:k), 'YData', x(1:k));
    set(yActualPlot, 'XData', t(1:k), 'YData', y(1:k));

    % Calculate and Update X and Y Tracking Error Plots
    xError = xd(1:k) - x(1:k);
    yError = yd(1:k) - y(1:k);
    set(xErrorPlot, 'XData', t(1:k), 'YData', xError);
    set(yErrorPlot, 'XData', t(1:k), 'YData', yError);

    drawnow
end

