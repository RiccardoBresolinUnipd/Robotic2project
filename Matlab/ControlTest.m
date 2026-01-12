clear; close all; clc;
addpath("./ControlUtils/")

%% ------------------ Parameters ----------------------
dt = 0.001;          
T  = 30;            % time
t  = 0:dt:T;

% Starting position
x_curr = 0;
y_curr = 0;
theta_curr = 0;

% Controller type:
%   - "linear"      = linear controller
%   - "non_linear"  = non-linear controller
%   - "c_on_CM"     = follow the (trajectory) CoM
%   - "c_on_B"      = follow the (trajectory) B-point
type = "c_on_B";

% Vectors to save the simulation
x = zeros(length(t),1);
y = zeros(length(t),1);
theta = zeros(length(t),1);
v_r = zeros(length(t),1);
w_r = zeros(length(t),1);

%% ---------------- Trajectory ----------------------
list = {"custom","square","triangle", "oval", "circle"};
[indx,tf] = listdlg('PromptString','Select random seed:', ...
    'SelectionMode','single','InitialValue',1, 'ListSize',[150,100], 'ListString',list);

[xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x_curr, y_curr, t,list{indx});

%% ---------------- Simulation ----------------------
for k = 1:length(t)
    
    % State
    x(k) = x_curr;
    y(k) = y_curr;
    theta(k) = theta_curr;

    % Desired trajectory
    x_d = xd(k);
    y_d = yd(k);
    dx_d = dxd(k);
    dy_d = dyd(k);
    ddx_d = ddxd(k);
    ddy_d = ddyd(k);

    % Differential flatness
    [x_d, y_d, theta_d, vd, wd] = differential_flatness(x_d, y_d, dx_d, dy_d, ddx_d, ddy_d);
    % theta_d = wrapToPi(theta_d);

    % Tracking controller
    [v, w] = tracking_controller(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, vd, wd, type);
    v_r(k) = v;
    w_r(k) = w;

    % unicycle
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);
    % theta_curr = wrapToPi(theta_curr);

end
%% -------------------- PLOT RESULTS ---------------------------
fig = figure('Name', 'Robot Tracking', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);

% Subplot 1: Maze (Left Left)
axesMaze = subplot(2, 2, 1);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
axis(axesMaze, "square");
plot(axesMaze, xd, yd, 'r--', 'LineWidth',1.5);
plot(axesMaze, x, y, 'b', 'LineWidth',2);
legend("Desired Trajectory", "Robot");
title("Tracking");
xlabel("x"); ylabel("y");

axesMaze = subplot(2, 2, 3);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, w_r); legend("w")

axesMaze = subplot(2, 2, 2);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, x - xd', t, y-yd'); legend("x", "y") %, "xdes", "ydes")

axesMaze = subplot(2, 2, 4);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, theta);