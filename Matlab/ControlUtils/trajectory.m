function [xd, yd, dxd, dyd, ddxd, ddyd] = trajectory(x, y, t)

% ---- spline points -----
Px = [x 2 2.3 4 4.3 5.5   7   9]; 
Py = [y 0  2  2 -3  -3  -2.5 -3];   

s_control = linspace(0,1,length(Px));   
s = linspace(0,1,length(t));    

ppx = spline(s_control, Px);    
ppy = spline(s_control, Py);    

xd = ppval(ppx, s);             
yd = ppval(ppy, s);             

% ---- derivatives (central difference) -----
dt = s(2) - s(1);

dxd = [0, (xd(3:end)-xd(1:end-2))/(2*dt), 0];
dyd = [0, (yd(3:end)-yd(1:end-2))/(2*dt), 0];

ddxd = [0, (xd(3:end)-2*xd(2:end-1)+xd(1:end-2))/dt^2, 0];
ddyd = [0, (yd(3:end)-2*yd(2:end-1)+yd(1:end-2))/dt^2, 0];

end
