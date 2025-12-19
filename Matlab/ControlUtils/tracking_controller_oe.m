function [v, w, y_real, y_des] = tracking_controller_oe(xd, yd, thetad, x, y, theta, vd, wd, linear)

eps_v = 1e-6;
b = 0.2;

T = [cos(theta), -b*sin(theta);
    sin(theta),   b*cos(theta)];

Tinv = [cos(theta), sin(theta);
    -sin(theta)/b,  cos(theta)/b];

y_real = [x; y] + b*[cos(theta); sin(theta)];
y_des = [xd; yd] + 0*[cos(thetad); sin(thetad)];
yd_des = T * [vd; wd];

K = [1; 1] * 2;

u = K .* (y_des - y_real); % + yd_des;

% ---- input transformation ----
v = Tinv(1,:) * u;
w = Tinv(2,:) * u;
end
