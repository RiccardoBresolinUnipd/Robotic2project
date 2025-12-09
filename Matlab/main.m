clear; close all; clc;

%% ------------------ Parameters ----------------------
dt = 0.01;          
T  = 30;            % time
t  = 0:dt:T;

% Starting position
x_curr = 0;
y_curr = 0;
theta_curr = 0;

% controller
linear = true;      % true = linear controller  - false = non-linear controller

% Vectors to save the simulation
x = zeros(length(t),1);
y = zeros(length(t),1);
theta = zeros(length(t),1);

%% ---------------- Trajectory ----------------------
[xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x_curr, y_curr, t);

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

    % Tracking controller
    [v, w] = tracking_controller(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, vd, wd, linear);

    % unicycle
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

end

save('dati.mat', 'x', 'y', 'xd', 'yd');

%% -------------------- PLOT RISULTATI ---------------------------
figure; hold on; grid on; axis equal;
plot(xd, yd, 'r--', 'LineWidth',1.5);
plot(x, y, 'b', 'LineWidth',2);
legend("Traiettoria desiderata", "Robot");
title("Tracking");
xlabel("x"); ylabel("y");

