function [v, w] = tracking_controller(xd, yd, thetad, x, y, theta, vd, wd, linear)

eps_v = 1e-6;

% ---- trajectory error in body frame ----
e(1) =  (xd-x)*cos(theta) + (yd-y)*sin(theta);
e(2) = -(xd-x)*sin(theta) + (yd-y)*cos(theta);
e(3) = thetad-theta;

    if linear
        % Linear
        epsilon = 1/sqrt(2);
        a = 30;
        K1 = 2*epsilon*a;
        K2 = (a^2 - wd^2)/(vd + eps_v);
        K3 = K1;

        u1 = -K1*e(1);
        u2 = -K2*e(2) - K3*e(3);
    else
        % Non linear
        ro = 1/sqrt(2);
        b = 3000;
        K1 = 2*ro*sqrt(b*vd+wd^2);
        K2 = b;
        K3 = K1;

        u1 = -K1*e(1);
        u2 = -K2*e(2)*vd*sin(e(3))/(e(3) + eps_v) - K3*e(3);
    end

% ---- input transformation ----
v = vd*cos(e(3)) - u1;
w = wd - u2;
end
