clear; close all; clc;
addpath("./ControlUtils/")

%% ------------------ Parameters ----------------------
dt = 0.001;
T  = 30;            % time
t  = 0:dt:T;

% controller options
opts.b = 0.2;
opts.K = 2;
opts.type = "c_on_B"; % "c_on_B", "c_on_CM"
b = 0.2;

% Starting position
x_curr = 0;
y_curr = 0;
theta_curr = 0;

% Vectors to save the simulation
x = zeros(length(t),1);
y = zeros(length(t),1);
theta = zeros(length(t),1);
thetad = zeros(length(t),1);
v_r = zeros(length(t),1);
w_r = zeros(length(t),1);

%% ---------------- Trajectory ----------------------
list = {"custom","square","triangle", "oval", "circle"};
% [indx,tf] = listdlg('PromptString','Select random seed:', ...
%     'SelectionMode','single','InitialValue',1, 'ListSize',[150,100], 'ListString',list);
indx = 2;
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
    thetad(k) = theta_d;
    % Tracking controller
    [v, w] = tracking_controller_oe(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, ...
        dx_d, dy_d,wd, type=opts.type, b=opts.b, K=opts.K);
    v_r(k) = v;
    w_r(k) = w;

    % unicycle
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

end


%% -------------------- PLOT RESULTS ---------------------------
fig = figure('Name', 'Robot Tracking', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);

% Subplot 1: Maze (Left Left)
axesMaze = subplot(2, 2, 1);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
axis(axesMaze, "equal");
plot(axesMaze, xd, yd, 'r--', 'LineWidth',1);
plot(axesMaze, x, y, 'g', 'LineWidth',1.5);
plot(axesMaze, x + b * cos(theta), y + b * sin(theta) , 'y', 'LineWidth',2);
plot(axesMaze, xd' + b * cos(thetad), yd' + b * sin(thetad));
legend("Desired Trajectory", "Robot", "B-robot", "B-des");
title("Tracking");
xlabel("x"); ylabel("y");

axesMaze = subplot(2, 2, 3);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, w_r); legend("x", "y")

axesMaze = subplot(2, 2, 2);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, x - xd', t, y -yd'); legend("x", "y")

axesMaze = subplot(2, 2, 4);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, xd, t, x); legend("des","real")