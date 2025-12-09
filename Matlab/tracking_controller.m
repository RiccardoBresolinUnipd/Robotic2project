function [v, w] = tracking_controller(xd, yd, thetad, x, y, theta, vd, wd, linear)

eps_v = 1e-6;

    if linear
        % Lineaer
        epsilon = 1/sqrt(2);
        a = 30;
        K1 = 2*epsilon*a;
        K2 = (a^2 - wd^2)/(vd + eps_v);
        K3 = K1;
        
        u1 = -K1*((xd-x)*cos(theta) + (yd-y)*sin(theta));
        u2 = -K2*(-(xd-x)*sin(theta) + (yd-y)*cos(theta)) - K3*(thetad-theta);
        v = vd*cos(thetad - theta) -u1;
        w = wd -u2;
    else
        % Non linear
        ro = 1/sqrt(2);
        b = 30;
        K1 = 2*ro*sqrt(b*vd+wd^2);
        K2 = b;
        K3 = K1;
        
        u1 = -K1*((xd-x)*cos(theta) + (yd-y)*sin(theta));
        u2 = -K2*(-(xd-x)*sin(theta) + (yd-y)*cos(theta))*vd*(sin(thetad-theta))/(thetad-theta + eps_v) - K3*(thetad-theta);
        v = vd*cos(thetad - theta) - u1;
        w = wd - u2;
    end

end
