clear; close all; clc;
addpath("./ControlUtils/")

%% ------------------ Parameters ----------------------
dt = 0.001;
T  = 30;            % time
t  = 0:dt:T;

% controller options
set_bpoints = [0.2]; % [0.2 0.5 0.75];

opts.b = set_bpoints(1);
opts.K = 20;
opts.type = "c_on_CM"; % "c_on_B", "c_on_CM"


% Starting position
x_curr = 0;
y_curr = 0;
theta_curr = 0;

% Vectors to save the simulation
x       = zeros(length(t),length(set_bpoints));
y       = zeros(length(t),length(set_bpoints));
theta   = zeros(length(t),length(set_bpoints));
thetad  = zeros(length(t),length(set_bpoints));
v_r     = zeros(length(t),length(set_bpoints));
w_r     = zeros(length(t),length(set_bpoints));

%% ---------------- Trajectory ----------------------
list = {"custom","square","triangle", "oval", "circle"};
% [indx,tf] = listdlg('PromptString','Select random seed:', ...
%     'SelectionMode','single','InitialValue',1, 'ListSize',[150,100], 'ListString',list);
indx = 2;
[xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x_curr, y_curr, t,list{indx});

for kbp=1:length(set_bpoints)
    opts.b = set_bpoints(kbp);
    %% ---------------- Simulation ----------------------
    for k = 1:length(t)

        % State
        x(k, kbp) = x_curr;
        y(k, kbp) = y_curr;
        theta(k, kbp) = theta_curr;

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
        thetad(k, kbp) = theta_d;
        % Tracking controller
        [v, w] = tracking_controller_oe(x_d, y_d, theta_d, x_curr, y_curr, theta_curr, ...
            dx_d, dy_d,wd, type=opts.type, b=opts.b, K=opts.K);
        v_r(k, kbp) = v;
        w_r(k, kbp) = w;

        % unicycle
        [x_curr, y_curr, theta_curr] = unicycle_model(x_curr, y_curr, theta_curr, v, w, dt);

    end

end

%% save
save(list{indx}+"_"+opts.type, "x","y","theta", "v_r", "w_r", ...
    "xd", "yd", "dxd", "dyd", "ddxd", "ddyd", "thetad", "t", "dt", "opts", "set_bpoints", "list", "indx")

return
%%
if length(set_bpoints) ~= 1
    figure(Name="Comparison of B values")
    hold('on');
    grid('on');
    axis("equal");
    plot(xd, yd, '--');
    plot(x, y, LineWidth=1.5);
    legend(["Desired" string(set_bpoints)], Location="northwest");
    xlabel("x"); ylabel("y");
    switch opts.type
        case "c_on_CM"
            xlim([1.7, 2.1]); ylim([-0.1, 0.3]);
        case "c_on_B"
            xlim([1.2, 2.1]); ylim([-0.1, 0.8]);
    end
end
return
%% -------------------- PLOT RESULTS ---------------------------

x       = x(:,1);
y       = y(:,1);
theta   = theta(:,1);
thetad  = thetad(:,1);
v_r     = v_r(:,1);
w_r     = w_r(:,1);

fig = figure('Name', 'Robot Tracking', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);

% Subplot 1: Maze (Left Left)
axesMaze = subplot(2, 2, 1);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
axis(axesMaze, "equal");
plot(axesMaze, xd, yd, '--');
plot(axesMaze, x, y, LineWidth=1);
switch opts.type 
    case "c_on_CM"
        xlim(axesMaze, [1.8, 2.3]); ylim(axesMaze, [-0.1, 0.4]);
        plot(axesMaze, xd' + opts.b * cos(thetad), yd' + opts.b * sin(thetad), "--");
    case "c_on_B"
        xlim(axesMaze, [1.6, 2.1]); ylim(axesMaze, [-0.1, 0.4]);
        plot(axesMaze, xd', yd', "--");
end
plot(axesMaze, x + opts.b * cos(theta), y + opts.b * sin(theta), LineWidth=1);
legend("Desired Trajectory", "Robot", "B-des", "B-Robot", Location="northwest");
title("Tracking");
xlabel("x"); ylabel("y");

axesMaze = subplot(2, 2, 3);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, w_r); legend("w")
% xlim(axesMaze, [5, 8]);

axesMaze = subplot(2, 2, 2);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, x - xd', t, y -yd'); legend("x", "y")
% xlim(axesMaze, [5, 8]);

axesMaze = subplot(2, 2, 4);
hold(axesMaze, 'on');
grid(axesMaze, 'on');
plot(axesMaze, t, x + opts.b * cos(theta) - xd' , t, y + opts.b * sin(theta) -yd'); legend("x", "y")