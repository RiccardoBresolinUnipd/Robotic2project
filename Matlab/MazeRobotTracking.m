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
T  = 25;            % tracking time
t  = 0:dt:T;

% Controller
linear = true;      % true = linear controller  - false = non-linear controller
                    % "c_on_CM" = controller on the CoM
                    % "c_on_B"  = controller on B-point

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

% set parking spot
target = struct("x", goalWorld(1), "y", goalWorld(2), "theta", 2*pi/3 );

%% Parse Path
% Parse Path to Match Simulation Time Points
[xd, yd, dxd, dyd, ddxd, ddyd] = ParsePath(plannedPathWorldSmooth, t, dt);

% find the (index relative to the) instant the trajectory enters the box
kt = find(xd > 10 & yd > 10, 1);
% cut tracking reference and time up to that instant
t = t(1:kt); T = t(end);
xd = xd(1:kt); dxd = dxd(1:kt); ddxd = ddxd(1:kt);
yd = yd(1:kt); dyd = dyd(1:kt); ddyd = ddyd(1:kt);
% Leave enough time to the regulation controller
pT = T + 10;        % parking time
pt = T:dt:pT;

% Get Starting Position from Path
x_curr = xd(1);
y_curr = yd(1);
theta_curr = atan2(dyd(1), dxd(1));

%% Simulation Setup

x = zeros(kt+length(pt),1);
y = zeros(kt+length(pt),1);
theta = zeros(kt+length(pt),1);

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
% parking target
[goalXgrid, goalYgrid] = UnicycleView(goalXgrid,goalYgrid, -target.theta, 1.2);
plot(axesMaze, goalXgrid, goalYgrid, 'r-', 'MarkerSize', 8, 'LineWidth', 2);

% Initialize Robot Plot (Yellow Dot - Like Pacman)
robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
robotPlot = plot(axesMaze, robotXgrid, robotYgrid, 'y-', 'MarkerSize', 1, ...
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

% Subplot 3: Rho and Theta Parking Errors (Bottom left)
parkingError = subplot(2, 2, 3);
hold(parkingError, 'on');
grid(parkingError, 'on');

% Zero reference line
plot(parkingError, [T, pT], [0, 0], 'k--', 'LineWidth', 1);

% Initialize error plots
rhoErrorPlot = plot(parkingError, T, 0, 'r-', 'LineWidth', 2);
thetaErrorPlot = plot(parkingError, T, 0, 'b-', 'LineWidth', 2);
xlabel(parkingError, 'Time (s)');
ylabel(parkingError, 'Tracking Error (m)');
title(parkingError, 'Rho and Theta Parking Errors');
legend(parkingError, 'Zero Reference', 'Rho Error', 'Theta Error', 'Location', 'ne');
xlim(parkingError, [T, pT]);
ylim(parkingError, [-1, 3.5]);

%% Tracking Simulation
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
    [v, w] = tracking_controller(x_d, y_d, theta_d, x_curr, y_curr, theta_curr,vd,wd,linear);

    % Update X and Y Position Plots
    set(xActualPlot, 'XData', t(1:k), 'YData', x(1:k));
    set(yActualPlot, 'XData', t(1:k), 'YData', y(1:k));

    % Calculate and Update X and Y Tracking Error Plots
    xError = xd(1:k) - x(1:k);
    yError = yd(1:k) - y(1:k);
    set(xErrorPlot, 'XData', t(1:k), 'YData', xError);
    set(yErrorPlot, 'XData', t(1:k), 'YData', yError);

    % Unicycle Model
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

    % Update Robot Visualization
    robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
    robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
    [robXplot, robYplot] = UnicycleView(robotXgrid,robotYgrid, -theta_curr , 1);
    set(robotPlot, 'XData', robXplot, 'YData', robYplot);

    % Update Robot Traveled Path
    addpoints(robotPathLine, robotXgrid, robotYgrid);

    drawnow limitrate
end

%% Parking Simulation

gains = struct("kv",1, "kw", 3.5, "kd", 3);
for k = kt:kt+length(pt)
    % State
    x(k) = x_curr;
    y(k) = y_curr;
    theta(k) = theta_curr;

    % Parking Controller
    [v, w] = regulation_controller(x_curr,y_curr,theta_curr, target, gains);

    % Update Rho and Theta Error Plots
    set(rhoErrorPlot, 'XData', pt(1:k-kt)', 'YData', sqrt((x(kt+1:k)-target.x).^2 + (y(kt+1:k)-target.y).^2));
    set(thetaErrorPlot, 'XData', pt(1:k-kt)', 'YData', target.theta - theta(kt+1:k));

    % Unicycle Model
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

    % Update Robot Visualization
    robotXgrid = (x_curr - xLimits(1)) * mapRes + 0.5;
    robotYgrid = (yLimits(2) - y_curr) * mapRes + 0.5;
    [robXplot, robYplot] = UnicycleView(robotXgrid,robotYgrid, -theta_curr , 1);
    set(robotPlot, 'XData', robXplot, 'YData', robYplot);

    % Update Robot Traveled Path
    addpoints(robotPathLine, robotXgrid, robotYgrid);

    drawnow limitrate

end