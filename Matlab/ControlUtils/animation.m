clc; clear; close all;

%% Load the data
load('data.mat'); 

%% figure
figure;
hold on;
grid on;
axis equal;
axis([min(x)-0.1, max(x)+4.1, min(y)-2.1, max(y)+0.1]);
xlabel('X');
ylabel('Y');
Designed_tr = plot(nan, nan, 'r--', 'LineWidth', 1); 
set(Designed_tr, 'XData', xd, 'YData', yd);
title('Animation');

%% Obstacols
walls = [
    0.5, 0.5, 1.0, 2.0;
    3.0, -1.0, 0.8, 2.0;
    4.5, 0.0, 0.7, 2.5;
    5, -3.0, 0.2, 2.0;
    7, -4, 1.3, 1;
];

for i = 1:size(walls,1)
    rectangle('Position', walls(i,:), 'FaceColor', [0.5 0.5 0.5]);
end

%% Parcking box
goal = [9.0, -5.0, 4.0, 4.0];  % [x, y, length, height]
hGoal = rectangle('Position', goal, 'FaceColor', [1 1 1 0], 'EdgeColor','k', 'LineWidth', 2); 

%% Trajectory
hLine = plot(nan, nan, 'b-', 'LineWidth', 2); 
hPoint = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r'); 

%% Animation
for k = 1:length(x)
    set(hLine, 'XData', x(1:k), 'YData', y(1:k));

    px = x(k);
    py = y(k);

    set(hPoint, 'XData', px, 'YData', py);

    drawnow;           
    pause(0.00);       
end
