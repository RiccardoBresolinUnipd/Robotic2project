function [v, w] = tracking_controller_oe(xd, yd, thetad, x, y, theta, vd, wd, type, opts)
arguments
    xd
    yd
    thetad
    x
    y
    theta
    vd
    wd
    type  {mustBeMember(type, ["c_on_B", "c_on_CM"])}
    opts.b = 0.2
    opts.K = 2
end

b = opts.b;

T = @(theta) [cos(theta), -b*sin(theta);
    sin(theta),   b*cos(theta)];

Tinv = [cos(theta), sin(theta);
    -sin(theta)/b,  cos(theta)/b];

y_real = [x; y] + T(theta) * [b; 0];

switch type
    case "c_on_B"
        % follow the virtual B-point of the trajectory
        y_des  = [xd; yd] + T(thetad) * [b; 0];
        yd_des = T(thetad) * [vd; wd];
    case "c_on_CM"
        % follow the (CoM of) the trajectory, i.e. the trajectory itself
        y_des  = [xd; yd];
        yd_des = T(thetad) * [vd; 0]; 
end

K = [1; 1] * opts.K;

u = K .* (y_des - y_real) + yd_des;

% ---- input transformation ----
v_tmp = Tinv * u;
v = v_tmp(1);
w = v_tmp(2);
end