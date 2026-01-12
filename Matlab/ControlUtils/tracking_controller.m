function [v, w] = tracking_controller(xd, yd, thetad, x, y, theta, vd, wd, type, opts)
arguments
    % desired states
    xd
    yd
    thetad
    % current states
    x
    y
    theta
    % desired inputs
    vd
    wd
    type  {mustBeMember(type, ["linear","non_linear", "c_on_B", "c_on_CM"])}
    opts.b = 0.2 % b-point distance
    opts.K = 2   % b-point controller gain
end

eps_v = 1e-6;

switch type
    case "linear"
        % ---- trajectory error in body frame ----
        e(1) =  (xd-x)*cos(theta) + (yd-y)*sin(theta);
        e(2) = -(xd-x)*sin(theta) + (yd-y)*cos(theta);
        e(3) = thetad-theta;

        % Linear
        epsilon = 1/sqrt(2);
        a = 30;
        K1 = 2*epsilon*a;
        K2 = (a^2 - wd^2)/(vd + eps_v);
        K3 = K1;

        % ---- Compute feedback ----
        u1 = -K1*e(1);
        u2 = -K2*e(2) - K3*e(3);
        % ---- input transformation ----
        v = vd*cos(e(3)) - u1;
        w = wd - u2;

    case "non_linear"
        % ---- trajectory error in body frame ----
        e(1) =  (xd-x)*cos(theta) + (yd-y)*sin(theta);
        e(2) = -(xd-x)*sin(theta) + (yd-y)*cos(theta);
        e(3) = thetad-theta;

        % Non linear
        ro = 1/sqrt(2);
        b = 3000;
        K1 = 2*ro*sqrt(b*vd+wd^2);
        K2 = b;
        K3 = K1;

        % ---- Compute feedback ----
        u1 = -K1*e(1);
        u2 = -K2*e(2)*vd*sin(e(3))/(e(3) + eps_v) - K3*e(3);
        % ---- input transformation ----
        v = vd*cos(e(3)) - u1;
        w = wd - u2;

    case "c_on_B"
        b = opts.b;
        % ---- Transformation Matrices ----
        T = @(theta) [cos(theta), -b*sin(theta);
            sin(theta),   b*cos(theta)];
        Tinv = [cos(theta), sin(theta);
            -sin(theta)/b,  cos(theta)/b];
        % ---- Robot B-point ----
        y_real = [x; y] + T(theta) * [b; 0];

        % follow the virtual B-point of the trajectory
        y_des  = [xd; yd] + T(thetad) * [b; 0];
        yd_des = T(thetad) * [vd; wd];

        % ---- Compute feedback ----
        K = [1; 1] * opts.K;
        u = K .* (y_des - y_real) + yd_des;
        % ---- input transformation ----
        v_tmp = Tinv * u;
        v = v_tmp(1);
        w = v_tmp(2);

    case "c_on_CM"
        b = opts.b;
        % ---- Transformation Matrices ----
        T = @(theta) [cos(theta), -b*sin(theta);
            sin(theta),   b*cos(theta)];
        Tinv = [cos(theta), sin(theta);
            -sin(theta)/b,  cos(theta)/b];
        % ---- Robot B-point ----
        y_real = [x; y] + T(theta) * [b; 0];

        % follow the (CoM of) the trajectory, i.e. the trajectory itself
        y_des  = [xd; yd];
        yd_des = T(thetad) * [vd; 0];

        % ---- Compute feedback ----
        K = [1; 1] * opts.K;
        u = K .* (y_des - y_real) + yd_des;
        % ---- input transformation ----
        v_tmp = Tinv * u;
        v = v_tmp(1);
        w = v_tmp(2);
end
end
