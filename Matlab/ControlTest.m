clear; close all; clc;
addpath("./ControlUtils/")

%% ------------------ Parameters ----------------------
dt = 0.001;          
T  = 30;            % time
t  = 0:dt:T;
N = length(t);

% Starting position
x_curr = 0;
y_curr = 0;
theta_curr = 0;

% controller
linear = true;      % true = linear controller  - false = non-linear controller

% Vectors to save the simulation
p   = struct("x", zeros(N,1), "th", zeros(N,1),"y", zeros(N,1), ...
             "dx", zeros(N,1), "dth", zeros(N,1),"dy", zeros(N,1));
pd  = p;
u   = struct("v", zeros(N,1), "w", zeros(N,1));
ud  = u;

%% ---------------- Trajectory ----------------------
% list = {"custom","square","triangle", "oval", "circle"};
% [indx,tf] = listdlg("Name","Trajectory",'PromptString','Select shape:', ...
%     'SelectionMode','single','InitialValue',1, 'ListSize',[150,100], 'ListString',list);

[pd.x, pd.y, pd.dx, pd.dy, pd.ddx, pd.ddy] = trajectory(x_curr, y_curr, t, "square"); %list{indx});

%% ---------------- Simulation ----------------------
for k = 1:length(t)
    
    % State
    p.x(k) = x_curr; p.y(k) = y_curr; p.theta(k) = theta_curr;

    % Desired trajectory
    x_d     = pd.x(k);      y_d   = pd.y(k);
    dx_d    = pd.dx(k);     dy_d  = pd.dy(k);
    ddx_d   = pd.ddx(k);    ddy_d = pd.ddy(k);

    % Differential flatness
    [x_d, y_d, theta_d, vd, wd] = differential_flatness(x_d, y_d, dx_d, dy_d, ddx_d, ddy_d);
    pd.th(k) = theta_d;

    % Tracking controller
    [v, w, y_real, y_des] = tracking_controller_oe(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, vd, wd, linear);
    v_r(k,:) = y_real';
    w_r(k,:) = y_des';

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
axis(axesMaze, "equal");
plot(axesMaze, xd, yd, 'r--', 'LineWidth',1.5);
plot(axesMaze, p.x, p.y, 'g', 'LineWidth',2);
legend("Desired Trajectory", "Robot");
title("Tracking center");
xlabel("x"); ylabel("y");

axesMaze = subplot(2, 2, 3);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
axis(axesMaze, "equal");
% plot(axesMaze, t, v_r(:,2) - w_r(:,2), t, v_r(:,1) - w_r(:,1)); legend("y", "x")
plot(axesMaze, w_r(:,1), w_r(:,2),v_r(:,1), v_r(:,2)); legend("des", "real")
title("Tracking B point");

axesMaze = subplot(2, 2, 2);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, p.x - xd', t, p.y-yd'); legend("x", "y") %, "xdes", "ydes")

axesMaze = subplot(2, 2, 4);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, wrapToPi(wrapToPi(p.theta) - pd.theta)); legend("real", "des");
% plot(axesMaze, t, p.x, t, p.y, t, xd,t,yd); legend("x", "y","xdes", "ydes")