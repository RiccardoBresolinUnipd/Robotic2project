function [v, w] = tracking_controller_oe(xd, yd, thetad, x, y, theta, vd, wd, linear)

eps_v = 1e-6;
b = 0.75;

T = [cos(theta), -b*sin(theta);
              sin(theta),   b*cos(theta)];

Tinv = [cos(theta), sin(theta);
    -sin(theta)/b,  cos(theta)/b];

y_real = [x; y] + b*[cos(theta); sin(theta)];
y_des = [xd; yd] + b*[cos(thetad); sin(thetad)];
yd_des = T * [vd; wd];

K = [1; 1] * 50;

u = K .* (y_des - y_real) + yd_des;

% ---- input transformation ----
v = Tinv(1,:) * u;
w = Tinv(2,:) * u;
end
