function [xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x, y, t, type)
arguments
    x       (1,1) double
    y       (1,1) double
    t       (1,:) double
    type    (1,:) {mustBeMember(type, ...
        ["custom","square","triangle", "oval", "circle"])} = "custom"
end

switch type
    case "custom"
        Px = [x 2 2.3 4 4.3 5.5   7   9];
        Py = [y 0  2  2 -3  -3  -2.5 -3];
        method = "spline";
    case "square"
        Px = [x 2 2 -2 -2 0];
        Py = [y 0 4  4  0 0];
        method = "linear";
    case "triangle"
        Px = [x 0.5    0       -0.5  0] * 2;
        Py = [y 0   sqrt(3)/2   0   0] * 2;
        method = "linear";
    case "oval"
        Px = [x 1 0 -1 0];
        Py = [y 1 2 1 0];
        method = "spline";
    case "circle"
        th = linspace(0, 2*pi, 100);
        R = 2;
        Px = R * cos(th - pi/2);
        Py = R * sin(th - pi/2) + R;
        method = "spline";
end
s_control = linspace(t(1), t(end), length(Px));

pp_x = interp1(s_control, Px, method, 'pp');
pp_y = interp1(s_control, Py, method, "pp");

xd = ppval(pp_x, t);
yd = ppval(pp_y, t);

dxd = ppval(fnder(pp_x, 1), t);
dyd = ppval(fnder(pp_y, 1), t);

ddxd = ppval(fnder(pp_x, 2), t);
ddyd = ppval(fnder(pp_y, 2), t);
end