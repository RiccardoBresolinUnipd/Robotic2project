clear; close all; clc;
addpath("./ControlUtils/")
save = false;

%% ------------------ Parameters ----------------------
dt = 0.001;
T  = 15;            % time
t  = 0:dt:T;

% input_target = inputdlg({'X','Y','THETA'},'Target', [1 50; 1 12; 1 7]);
% target = cell2struct(cellfun(@str2num,input_target, UniformOutput=false) ,["x", "y", "theta"]);
targ_vec(1).x =  1; targ_vec(1).y =  1; targ_vec(1).theta = pi*3/2;
targ_vec(2).x = -1; targ_vec(2).y =  0; targ_vec(2).theta = pi;
targ_vec(3).x = -1; targ_vec(3).y =  0; targ_vec(3).theta = 0;
targ_vec(4).x = -1; targ_vec(4).y = -1; targ_vec(4).theta = 0;
targ_vec(5).x = -1; targ_vec(5).y =  1; targ_vec(5).theta = 0;
targ_vec(6).x =  1; targ_vec(6).y =  1; targ_vec(6).theta = 0;

gains = struct("kv",1, "kw", 3.5, "kd", 3);

for select=1:length(targ_vec)

    % Starting position
    x_curr = 0;
    y_curr = 0;
    theta_curr = 0;
    % Vectors to save the simulation
    x = zeros(length(t),1);
    y = zeros(length(t),1);
    theta = zeros(length(t),1);

    %% ---------------- Simulation ----------------------
    target = targ_vec(select);
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
    figure(Name="Results")
    traj = subplot(2,2,[1 3]); hold(traj,"on"); grid(traj,"on"); axis(traj,"equal");
    xlim([min(x)-0.2 max(x)+0.2]); ylim([min(y)-0.2 max(y)+0.2]);

    plot(traj, x, y, LineWidth=2);

    [startX, startY] = UnicycleView(x(1), y(1), theta(1), 0.1);
    plot(traj, startX, startY, LineWidth=1.5);

    [targX, targY] = UnicycleView(target.x,target.y, target.theta , 0.1);
    plot(traj, targX, targY, LineWidth=1.5, Color="red");

    [endingX, endingY] = UnicycleView(x(end),y(end), theta(end) , 0.1);
    plot(traj, endingX, endingY, LineWidth=1.5);

    legend(traj, "Trajectory", "Starting point", "Target point", "Ending position", Location="north");
    title(traj, "Regulation"); xlabel(traj, "x [m]"); ylabel(traj, "y[m]");

    % Cartesian Error
    carterr = subplot(2,2,2); hold(carterr,"on"); grid(carterr,"on");
    plot(carterr, t, target.x - x, t, target.y - y, LineWidth=2);
    legend(carterr, "x", "y");
    title(carterr, "Cartesian Errors"); xlabel(carterr, "t [s]"); ylabel(carterr, "e [m]");

    % Orientation Error
    angerr = subplot(2,2,4); hold(angerr,"on"); grid(angerr,"on");
    plot(angerr, t, target.theta - theta, LineWidth=2);
    title(angerr, "Orientation Error"); xlabel(angerr, "t [s]"); ylabel(angerr, "e [rad]");

    if save
        exportgraphics(gcf, ...
            ".\corrected_images\" + "regulation_" + string(select) +".png", Resolution=300);
    end
end