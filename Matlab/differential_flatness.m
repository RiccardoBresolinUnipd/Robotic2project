function [xd, yd, thetad, vd, wd] = differential_flatness(x, y, dx, dy, ddx, ddy)

eps_v = 1e-6;

xd = x;
yd = y;
thetad = atan2(dy, dx);
vd = sqrt(dx^2 + dy^2);
wd = (dx*ddy - ddx*dy)/(dx^2 + dy^2 + eps_v);

end