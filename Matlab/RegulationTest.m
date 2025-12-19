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

input_target = inputdlg({'X','Y','THETA'},'Target', [1 50; 1 12; 1 7]);
target = cell2struct(cellfun(@str2num,input_target, UniformOutput=false) ,["x", "y", "theta"]);
gains = struct("kv",5, "kw", 5, "kd", 2);

% Vectors to save the simulation
x = zeros(length(t),1);
y = zeros(length(t),1);
theta = zeros(length(t),1);

%% ---------------- Simulation ----------------------
for k = 1:length(t)
    
    % State
    x(k) = x_curr;
    y(k) = y_curr;
    theta(k) = theta_curr;

    [v, w] = regulation_controller(x_curr,y_curr,theta_curr, target, gains);
    % unicycle
    [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);
end

%% -------------------- PLOT RESULTS ---------------------------
figure; hold on; grid on; axis equal;
plot(x, y, 'b', 'LineWidth',2);
plot(target.x, target.y, "ro");
title("Tracking");
xlabel("x"); ylabel("y");

figure(Name="pos"); plot(t, x, t, y); yline(target.x);yline(target.y); legend("x", "y")
figure(Name="theta"); plot(t, theta); yline(target.theta);
figure(Name="rho"); plot(t, sqrt((x - target.x).^2 + (y - target.y).^2))
