function [robotX, robotY] = UnicycleView(x, y, theta, scale)
arguments
    x       (1,1) double
    y       (1,1) double
    theta   (1,1) double
    scale   (1,1) double {mustBePositive(scale)} = 0.3
end
triangle = [1, -0.5, -0.5, 1;
            0, 0.5, -0.5, 0] * scale;

% rotation matrix
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];

% apply roto-translation
rotatedPoints = R * triangle + [x;y];

robotX = rotatedPoints(1,:);
robotY = rotatedPoints(2,:);

end