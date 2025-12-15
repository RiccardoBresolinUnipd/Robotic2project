function [xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x, y, t, type)
arguments
    x       (1,1) double
    y       (1,1) double
    t       (1,:) double
    type    (1,:) {mustBeMember(type, ...
        ["custom","square","triangle", "oval"])} = "custom"
end

s = linspace(0,1,length(t));

switch type
    case "custom"
        % ---- spline points -----
        Px = [x 2 2.3 4 4.3 5.5   7   9];
        Py = [y 0  2  2 -3  -3  -2.5 -3];
        s_control = linspace(0,1,length(Px));
        ppx = griddedInterpolant(s_control, Px, 'spline');
        ppy = griddedInterpolant(s_control, Py, 'spline');
    case "square"
        Px = [x 2 2 -2 -2 0];
        Py = [y 0 4 4 0 0];
        s_control = linspace(0,1,length(Px));
        ppx = griddedInterpolant(s_control, Px, 'linear');
        ppy = griddedInterpolant(s_control, Py, 'linear');
    case "triangle"
        Px = [x 0.5    0       -0.5  0];
        Py = [y 0   sqrt(3)/2   0   0];
        s_control = linspace(0,1,length(Px));
        ppx = griddedInterpolant(s_control, Px, 'linear');
        ppy = griddedInterpolant(s_control, Py, 'linear');
    case "oval"
        Px = [x 1 0 -1 0];
        Py = [y 1 2 1 0];
        s_control = linspace(0,1,length(Px));
        ppx = griddedInterpolant(s_control, Px, 'spline');
        ppy = griddedInterpolant(s_control, Py, 'spline');
end

% ---- evaluation of the interpolants ----
xd = ppx(s);
yd = ppy(s);

% ---- derivatives (central difference) -----
dt = s(2) - s(1);

dxd = [0, (xd(3:end)-xd(1:end-2))/(2*dt), 0];
dyd = [0, (yd(3:end)-yd(1:end-2))/(2*dt), 0];

ddxd = [0, (xd(3:end)-2*xd(2:end-1)+xd(1:end-2))/dt^2, 0];
ddyd = [0, (yd(3:end)-2*yd(2:end-1)+yd(1:end-2))/dt^2, 0];

end
