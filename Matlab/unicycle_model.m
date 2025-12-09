function [x, y, theta] = unicycle_model(x, y, theta, v, omega, dt)

x     = x + dt * v*cos(theta);
y     = y + dt * v*sin(theta);
theta = theta + dt * omega;

end
