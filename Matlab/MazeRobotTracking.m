clear
close all
clc
addpath("./MazeSolveUtils/")
addpath("./ControlUtils/")

list = {'4','15','20','25'};
[indx,tf] = listdlg('PromptString','Select random seed:', ...
    'SelectionMode','single','InitialValue',1, 'ListSize',[150,100], 'ListString',list);
global RANDOM_SEED
RANDOM_SEED = str2num(list{indx});
rng(RANDOM_SEED)

%% Parameters
dt = 0.01;
T  = 25;            % time
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
    CreateMazeAndSelectPoints(mazeProps, "fixed");
goalWorld = grid2world(mazeMap,goalGrid);

% Solve Maze Using A* Algorithm
[pathGrid, plannedPathWorldSmooth] = SolveMazeAStar(mazeMap, startGrid, goalGrid);

close(figureMaze);

%% Parse Path
% Parse Path to Match Simulation Time Points
[xd, yd, dxd, dyd, ddxd, ddyd] = ParsePath(plannedPathWorldSmooth, t, dt);

lastT = find(xd > 10 & yd > 10, 1);
xd = xd(1:lastT);
yd = yd(1:lastT);

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

% Subplot 1: Maze (Left Left)
axesMaze = subplot(2, 2, 1);
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

% Extract Start/Goal in Grid Coordinates
startXgrid = startGrid(2);
startYgrid = startGrid(1);

goalXgrid = goalGrid(2);
goalYgrid = goalGrid(1);

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
xDesiredPlot = plot(axesPos, t(1:lastT), xd, 'r--', 'LineWidth', 1.5);
xActualPlot = plot(axesPos, t(1), x_curr, 'r-', 'LineWidth', 2);

% Y Position - Desired and Actual
yDesiredPlot = plot(axesPos, t(1:lastT), yd, 'b--', 'LineWidth', 1.5);
yActualPlot = plot(axesPos, t(1), y_curr, 'b-', 'LineWidth', 2);
xlabel(axesPos, 'Time (s)');
ylabel(axesPos, 'Position (m)');
title(axesPos, 'X and Y Position Trajectories');
legend(axesPos, 'X Desired', 'X Actual', 'Y Desired', 'Y Actual', 'Location', 'nw');
xlim(axesPos, [0, t(lastT)]);

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
xlim(axesError, [0, t(lastT)]);
ylim(axesError, [-0.35, 0.35]);

% Subplot 3: Rho and Theta Parking Errors (Bottom left)
parkingError = subplot(2, 2, 3);
hold(parkingError, 'on');
grid(parkingError, 'on');

% Zero reference line
plot(parkingError, [t(lastT+1), T], [0, 0], 'k--', 'LineWidth', 1);

% Initialize error plots
rhoErrorPlot = plot(parkingError, t(lastT+1), 0, 'r-', 'LineWidth', 2);
thetaErrorPlot = plot(parkingError, t(lastT+1), 0, 'b-', 'LineWidth', 2);
xlabel(parkingError, 'Time (s)');
ylabel(parkingError, 'Tracking Error (m)');
title(parkingError, 'Rho and Theta Parking Errors');
legend(parkingError, 'Zero Reference', 'Rho Error', 'Theta Error', 'Location', 'ne');
xlim(parkingError, [t(lastT+1), T]);
ylim(parkingError, [-1, 3.5]);

%% Simulation
for k = 1:length(t)

    % State
    x(k) = x_curr;
    y(k) = y_curr;
    theta(k) = theta_curr;

    if (x_curr > 10 ) && ( y_curr > 10)
        % parking target
        target = struct("x", goalWorld(1), "y", goalWorld(2), "theta", 2*pi/3 );
        gains = struct("kv",5, "kw", 8, "kd", 2);

        % Parking Controller
        [v, w] = regulation_controller(x_curr,y_curr,theta_curr, target, gains);
        
        % Update Rho and Theta Error Plots
        set(rhoErrorPlot, 'XData', t(lastT+1:k), 'YData', sqrt((x(lastT+1:k)-target.x).^2 + (y(lastT+1:k)-target.y).^2));
        set(thetaErrorPlot, 'XData', t(lastT+1:k), 'YData', target.theta - theta(lastT+1:k));
    else
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

        % Update X and Y Position Plots
        set(xActualPlot, 'XData', t(1:k), 'YData', x(1:k));
        set(yActualPlot, 'XData', t(1:k), 'YData', y(1:k));

        % Calculate and Update X and Y Tracking Error Plots
        xError = xd(1:k) - x(1:k);
        yError = yd(1:k) - y(1:k);
        set(xErrorPlot, 'XData', t(1:k), 'YData', xError);
        set(yErrorPlot, 'XData', t(1:k), 'YData', yError);
    end

    % Unicycle Model
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

    % Update Robot Visualization
    robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
    robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
    set(robotPlot, 'XData', robotXgrid, 'YData', robotYgrid);

    % Update Robot Traveled Path
    addpoints(robotPathLine, robotXgrid, robotYgrid);

    drawnow limitrate
end

